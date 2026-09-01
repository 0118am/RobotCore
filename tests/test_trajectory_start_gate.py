"""Trajectory time must remain gated until the operator presses Start."""

import inspect
from types import SimpleNamespace

import numpy as np

from robotcore_runtime.trajectory_command_node import TrajectoryCommandNode
from robotcore_runtime.tracking_experiment_node import TrackingExperimentNode


def test_idle_target_is_explicitly_neutral_until_start():
    source = inspect.getsource(TrajectoryCommandNode.tick)

    assert 'published_trajectory_type = "idle"' in source
    assert 'published_control_mode = "idle"' in source
    assert 'published_phase = "idle"' in source


def test_automatic_targets_accept_either_selected_station_controller():
    compatible = TrajectoryCommandNode.control_mode_is_compatible

    assert compatible("hold", "altitude_hold") is True
    assert compatible("hold", "station_hold") is True
    assert compatible("hold", "station_hold_fast") is True
    assert compatible("hold", "rl_policy") is True
    for trajectory_type in (
        "circle",
        "racetrack",
        "straight_line",
        "spatial_lissajous",
    ):
        assert compatible(trajectory_type, "station_hold") is True
        assert compatible(trajectory_type, "station_hold_fast") is True
        assert compatible(trajectory_type, "rl_policy") is True
        assert compatible(trajectory_type, "altitude_hold") is False
    assert compatible("altitude_hold", "altitude_hold") is False
    assert compatible("station_hold", "station_hold") is False


def test_automatic_target_is_selected_only_by_validated_task_id():
    source = inspect.getsource(TrackingExperimentNode.execute)

    assert "request.task_id" in source
    assert "self.task_catalog.task(task_name)" in source
    assert 'controller = str(task["controller"])' in source
    assert "str(request.controller)" not in source
    assert "resolve_controller" not in source


def test_trajectory_uses_ekf_position_and_heading_with_imu_tilt_and_rates():
    source = inspect.getsource(TrajectoryCommandNode)
    body_callback = inspect.getsource(TrajectoryCommandNode.on_body_state)
    imu_callback = inspect.getsource(TrajectoryCommandNode.on_imu)

    assert 'self.declare_parameter("imu_topic", "/sensors/external_imu")' in source
    assert "QoSProfile(" in source
    assert "ReliabilityPolicy.BEST_EFFORT" in source
    assert "msg.pose.orientation" in body_callback
    assert "self.latest_position = position" in body_callback
    assert "self.latest_map_yaw" in body_callback
    assert "msg.orientation.w" in imu_callback
    assert "self.latest_quaternion = self.rpy_quaternion(" in imu_callback


def test_magnetic_yaw_jump_cannot_enter_the_station_heading():
    trajectory = object.__new__(TrajectoryCommandNode)
    trajectory.latest_position = None
    trajectory.initial_position = np.zeros(3)
    trajectory.latest_map_yaw = None
    trajectory.latest_quaternion = None
    trajectory.initial_quaternion = None
    trajectory.latest_angular_velocity = None

    body_attitude = trajectory.rpy_quaternion(-0.03, 0.02, np.deg2rad(22.24))
    body = SimpleNamespace(
        state_valid=True,
        position_estimated=False,
        pose=SimpleNamespace(
            position=SimpleNamespace(x=2.05, y=2.33, z=0.47),
            orientation=SimpleNamespace(
                w=body_attitude[0],
                x=body_attitude[1],
                y=body_attitude[2],
                z=body_attitude[3],
            ),
        ),
    )
    trajectory.on_body_state(body)

    disturbed_attitude = trajectory.rpy_quaternion(
        0.12, -0.08, np.deg2rad(-8.26)
    )
    imu = SimpleNamespace(
        header=SimpleNamespace(frame_id="base_link"),
        orientation_covariance=[0.0] * 9,
        orientation=SimpleNamespace(
            w=disturbed_attitude[0],
            x=disturbed_attitude[1],
            y=disturbed_attitude[2],
            z=disturbed_attitude[3],
        ),
        angular_velocity=SimpleNamespace(x=0.1, y=-0.2, z=0.3),
    )
    trajectory.on_imu(imu)

    assert np.allclose(
        trajectory.quaternion_to_rpy(trajectory.latest_quaternion),
        [0.12, -0.08, np.deg2rad(22.24)],
        atol=1e-10,
    )
    assert np.allclose(trajectory.latest_angular_velocity, [0.1, -0.2, 0.3])


def make_trajectory():
    trajectory = object.__new__(TrajectoryCommandNode)
    trajectory.started_ns = 1_000_000_000
    trajectory.tracking_started = False
    trajectory.envelope_checked = True
    trajectory.envelope_valid = True
    trajectory.motion_start_position = None
    trajectory.motion_start_quaternion = None
    trajectory.get_parameter = lambda name: SimpleNamespace(
        value={
            "trajectory_type": "hold",
            "control_mode": "station_hold",
            "center_z": 0.9,
        }[name]
    )
    trajectory.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=2_000_000_000)
    )
    return trajectory


def test_target_time_is_zero_before_start():
    trajectory = make_trajectory()

    assert trajectory.trajectory_time_s(9_000_000_000) == 0.0


def test_reset_starts_target_time_and_stop_freezes_it_again():
    trajectory = make_trajectory()
    trajectory.motion_start_position = np.asarray([1.0, 1.0, 0.9])
    trajectory.motion_start_quaternion = np.asarray([1.0, 0.0, 0.0, 0.0])
    response = SimpleNamespace(success=False, message="")

    trajectory.on_reset_scenario(None, response)
    assert response.success is True
    assert trajectory.tracking_started is True
    assert trajectory.envelope_checked is True
    assert trajectory.envelope_valid is True
    assert trajectory.trajectory_time_s(2_500_000_000) == 0.5

    trajectory.on_stop_scenario(None, response)
    assert trajectory.tracking_started is False
    assert trajectory.trajectory_time_s(8_000_000_000) == 0.0


def test_altitude_hold_reset_uses_configured_initial_height():
    trajectory = make_trajectory()
    trajectory.motion_start_position = np.asarray([1.2, 0.8, 0.62])
    trajectory.motion_start_quaternion = np.asarray([1.0, 0.0, 0.0, 0.0])
    trajectory.altitude_hold_z = None
    trajectory.altitude_heave_active = False
    trajectory.get_parameter = lambda name: SimpleNamespace(
        value={
            "trajectory_type": "hold",
            "control_mode": "altitude_hold",
            "center_z": 0.9,
        }[name]
    )
    response = SimpleNamespace(success=False, message="")

    trajectory.on_reset_scenario(None, response)

    assert response.success is True
    assert trajectory.altitude_hold_z == 0.9


def test_station_modes_reset_to_level_roll_pitch_and_keep_heading():
    for control_mode, expected_height in (
        ("station_hold", 0.9),
        ("station_hold_fast", 0.62),
        ("rl_policy", 0.62),
    ):
        trajectory = make_trajectory()
        trajectory.motion_start_position = np.asarray([1.2, 0.8, 0.62])
        trajectory.motion_start_quaternion = trajectory.rpy_quaternion(
            0.2, -0.1, 0.7
        )
        trajectory.station_target_position = None
        trajectory.station_target_quaternion = None
        trajectory.get_parameter = lambda name, mode=control_mode: SimpleNamespace(
            value={
                "trajectory_type": "hold",
                "control_mode": mode,
                "center_z": 0.9,
            }[name]
        )
        response = SimpleNamespace(success=False, message="")

        trajectory.on_reset_scenario(None, response)

        target_rpy = trajectory.quaternion_to_rpy(
            trajectory.station_target_quaternion
        )
        assert response.success is True
        assert trajectory.station_target_position[2] == expected_height
        assert np.allclose(target_rpy, [0.0, 0.0, 0.7])


def test_published_target_attitude_is_forced_level_except_yaw():
    source = inspect.getsource(TrajectoryCommandNode.tick)

    assert "orientation = tuple(self.level_heading_quaternion(orientation))" in source
    assert "angular_velocity = (0.0, 0.0, float(angular_velocity[2]))" in source
    assert "angular_acceleration = (0.0, 0.0, float(angular_acceleration[2]))" in source

    tilted = TrajectoryCommandNode.rpy_quaternion(0.31, -0.22, 1.17)
    level = TrajectoryCommandNode.level_heading_quaternion(tilted)
    assert np.allclose(
        TrajectoryCommandNode.quaternion_to_rpy(level),
        [0.0, 0.0, 1.17],
        atol=1e-12,
    )


def test_absolute_hold_uses_map_center_but_keeps_initial_attitude():
    trajectory = object.__new__(TrajectoryCommandNode)
    trajectory.initial_position = np.asarray([0.4, 1.2, 0.8], dtype=np.float64)
    trajectory.initial_quaternion = np.asarray(
        [0.70710678, 0.0, 0.0, 0.70710678], dtype=np.float64
    )
    parameters = {
        "relative_to_initial_pose": False,
        "center_x": 2.0,
        "center_y": 2.0,
        "center_z": 0.5,
        "amp_x": 0.0,
        "amp_y": 0.0,
        "amp_z": 0.0,
        "period_s": 30.0,
        "attitude_mode": "hold_initial",
    }
    trajectory.get_parameter = lambda name: SimpleNamespace(value=parameters[name])

    sample = trajectory.sample("hold", 0.0)
    attitude = trajectory.attitude_quaternion("hold", 0.0)

    assert sample.position == (2.0, 2.0, 0.5)
    assert np.allclose(attitude, trajectory.initial_quaternion)


def test_hold_initial_prefers_attitude_latched_after_heading_alignment():
    trajectory = object.__new__(TrajectoryCommandNode)
    trajectory.initial_quaternion = trajectory.rpy_quaternion(0.0, 0.0, 2.8)
    trajectory.latest_quaternion = trajectory.rpy_quaternion(0.0, 0.0, -0.9)
    trajectory.motion_start_quaternion = trajectory.rpy_quaternion(0.1, -0.1, -1.0)
    trajectory.get_parameter = lambda name: SimpleNamespace(
        value={"attitude_mode": "hold_initial", "relative_to_initial_pose": False}[name]
    )

    attitude = trajectory.attitude_quaternion("hold", 0.0)

    assert np.allclose(attitude, trajectory.motion_start_quaternion)


def make_altitude_envelope_trajectory():
    trajectory = object.__new__(TrajectoryCommandNode)
    trajectory.motion_start_position = np.asarray([2.4, 1.1, 0.88], dtype=np.float64)
    trajectory.motion_start_quaternion = trajectory.rpy_quaternion(0.1, -0.1, 2.4)
    trajectory.latest_angular_velocity = np.asarray([0.02, -0.01, 0.04])
    trajectory.initial_position = trajectory.motion_start_position.copy()
    parameters = {
        "trajectory_type": "hold",
        "control_mode": "altitude_hold",
        "relative_to_initial_pose": False,
        "pool_min_xyz": [0.0, 0.0, 0.0],
        "pool_max_xyz": [5.42, 3.73, 1.0],
        "trajectory_limits_configured": True,
        "max_linear_speed_mps": 0.45,
        "max_angular_speed_rps": 0.55,
        "attitude_min_rpy_deg": [-15.0, -15.0, -30.0],
        "attitude_max_rpy_deg": [15.0, 15.0, 30.0],
        "center_z": 0.9,
        "manual_vertical_speed_mps": 0.2,
    }
    trajectory.get_parameter = lambda name: SimpleNamespace(value=parameters[name])
    return trajectory


def test_altitude_hold_envelope_allows_current_absolute_map_yaw():
    trajectory = make_altitude_envelope_trajectory()

    assert trajectory.validate_scenario_envelope() is True
    assert trajectory.envelope_rejection_reason == ""


def test_altitude_hold_envelope_rejects_excess_tilt_and_rate():
    trajectory = make_altitude_envelope_trajectory()
    trajectory.motion_start_quaternion = trajectory.rpy_quaternion(0.4, 0.0, 2.4)

    assert trajectory.validate_scenario_envelope() is False
    assert "roll/pitch" in trajectory.envelope_rejection_reason

    trajectory.motion_start_quaternion = trajectory.rpy_quaternion(0.0, 0.0, 2.4)
    trajectory.latest_angular_velocity = np.asarray([0.0, 0.0, 0.8])
    assert trajectory.validate_scenario_envelope() is False
    assert "angular rate" in trajectory.envelope_rejection_reason


def test_move_to_hold_uses_minimum_jerk_then_stops_at_fixed_goal():
    trajectory = object.__new__(TrajectoryCommandNode)
    trajectory.initial_position = np.asarray([0.0, 0.0, 0.0], dtype=np.float64)
    trajectory.latest_position = trajectory.initial_position.copy()
    trajectory.motion_start_position = np.asarray([1.0, 2.0, 0.8], dtype=np.float64)
    parameters = {
        "relative_to_initial_pose": False,
        "center_x": 3.0,
        "center_y": 1.0,
        "center_z": 0.5,
        "amp_x": 0.0,
        "amp_y": 0.0,
        "amp_z": 0.0,
        "period_s": 30.0,
        "move_duration_s": 18.0,
    }
    trajectory.get_parameter = lambda name: SimpleNamespace(value=parameters[name])

    start = trajectory.sample("move_to_hold", 0.0)
    midpoint = trajectory.sample("move_to_hold", 9.0)
    goal = trajectory.sample("move_to_hold", 18.0)
    held = trajectory.sample("move_to_hold", 28.0)

    assert np.allclose(start.position, [1.0, 2.0, 0.8])
    assert np.allclose(start.velocity, np.zeros(3))
    assert np.allclose(start.acceleration, np.zeros(3))
    assert np.allclose(midpoint.position, [2.0, 1.5, 0.65])
    assert np.linalg.norm(midpoint.velocity) > 0.0
    assert np.allclose(midpoint.acceleration, np.zeros(3), atol=1e-12)
    assert np.allclose(goal.position, [3.0, 1.0, 0.5])
    assert np.allclose(goal.velocity, np.zeros(3), atol=1e-12)
    assert np.allclose(goal.acceleration, np.zeros(3), atol=1e-12)
    assert held == goal


def test_spatial_lissajous_is_three_dimensional_and_constant_point_one_mps():
    trajectory = object.__new__(TrajectoryCommandNode)
    parameters = {
        "amp_x": 1.5,
        "amp_y": 0.75,
        "amp_z": 0.25,
        "trajectory_speed_mps": 0.1,
        "trajectory_ramp_s": 10.0,
    }
    trajectory.get_parameter = lambda name: SimpleNamespace(value=parameters[name])

    _phases, cumulative = trajectory.lissajous_arc_table(1.5, 0.75, 0.25)
    lap_duration_s = float(cumulative[-1]) / 0.1
    samples = [
        trajectory.spatial_lissajous_kinematics(float(time_s))
        for time_s in np.linspace(10.0, 10.0 + lap_duration_s, 1001, endpoint=False)
    ]
    speeds = [np.linalg.norm(sample[1]) for sample in samples]
    heights = [sample[0][2] for sample in samples]

    assert np.allclose(speeds, 0.1, atol=1.0e-12)
    assert min(heights) < -0.249
    assert max(heights) > 0.249
    assert np.allclose(
        trajectory.spatial_lissajous_kinematics(0.0)[1], np.zeros(3)
    )


def test_automatic_prelude_moves_to_fixed_start_then_turns_in_place():
    trajectory = object.__new__(TrajectoryCommandNode)
    trajectory.initial_position = None
    trajectory.initial_quaternion = None
    trajectory.latest_position = np.asarray([1.4, 1.2, 0.75], dtype=np.float64)
    trajectory.latest_quaternion = trajectory.rpy_quaternion(0.05, -0.04, -2.63)
    trajectory.motion_start_position = trajectory.latest_position.copy()
    trajectory.motion_start_quaternion = trajectory.latest_quaternion.copy()
    parameters = {
        "relative_to_initial_pose": False,
        "center_x": 2.71,
        "center_y": 1.865,
        "center_z": 0.5,
        "amp_x": 0.8,
        "amp_y": 0.5,
        "amp_z": 0.2,
        "trajectory_speed_mps": 0.1,
        "trajectory_ramp_s": 5.0,
        "move_duration_s": 15.0,
        "hold_before_motion_s": 25.0,
        "publish_rate_hz": 60.0,
        "trajectory_type": "spatial_lissajous",
    }
    trajectory.get_parameter = lambda name: SimpleNamespace(value=parameters[name])
    tick_source = inspect.getsource(TrajectoryCommandNode.tick)

    turn_duration = (
        parameters["hold_before_motion_s"] - parameters["move_duration_s"]
    )
    move_start = trajectory.sample("move_to_trajectory_start", 0.0)
    move_midpoint = trajectory.sample("move_to_trajectory_start", 7.5)
    move_end = trajectory.sample("move_to_trajectory_start", 15.0)
    attitude_start = trajectory.attitude_quaternion(
        "move_to_trajectory_start", 0.0
    )
    attitude_end = trajectory.attitude_quaternion(
        "move_to_trajectory_start", turn_duration
    )
    turn_rates = [
        np.linalg.norm(
            trajectory.angular_velocity_at(
                "move_to_trajectory_start", time_s, 1.0 / 60.0
            )
        )
        for time_s in np.linspace(0.0, turn_duration, 121)
    ]
    path_start = trajectory.sample("spatial_lissajous", 0.0)
    path_start_attitude = trajectory.attitude_quaternion(
        "spatial_lissajous", 0.0
    )

    assert np.allclose(move_start.position, trajectory.motion_start_position)
    assert np.allclose(move_start.velocity, np.zeros(3))
    assert np.linalg.norm(move_midpoint.velocity) > 0.0
    assert np.allclose(move_end.position, [2.71, 1.865, 0.5])
    assert np.allclose(move_end.velocity, np.zeros(3), atol=1e-12)
    assert np.allclose(attitude_start, trajectory.motion_start_quaternion)
    assert np.isclose(abs(np.dot(attitude_end, path_start_attitude)), 1.0)
    assert turn_duration == 10.0
    assert max(turn_rates) < 0.60
    assert np.allclose(path_start.position, move_end.position)
    assert np.allclose(path_start.velocity, np.zeros(3))
    assert (
        "elif trajectory_type in AUTOMATIC_TRAJECTORY_TYPES and time_s < hold_s:"
        in tick_source
    )
    assert 'move_time_s = min(time_s, move_s)' in tick_source
    assert 'turn_time_s = max(0.0, time_s - move_s)' in tick_source
    assert 'self.sample("move_to_trajectory_start", move_time_s)' in tick_source
    assert '"move_to_trajectory_start", turn_time_s' in tick_source
    assert '"start_approach" if time_s < move_s else "heading_alignment"' in tick_source
    assert 'published_phase = "tracking"' in tick_source
    assert "msg.trajectory_phase = published_phase" in tick_source
    assert 'hasattr(msg, "trajectory_phase")' not in tick_source


def test_circle_racetrack_and_straight_line_are_periodic_planar_paths():
    trajectory = object.__new__(TrajectoryCommandNode)
    parameters = {
        "radius_m": 1.2,
        "amp_x": 2.3,
        "amp_y": 1.2,
        "trajectory_speed_mps": 0.1,
        "trajectory_ramp_s": 10.0,
    }
    trajectory.get_parameter = lambda name: SimpleNamespace(value=parameters[name])

    periods = {
        "circle": 2.0 * np.pi * parameters["radius_m"] / 0.1,
        "racetrack": (
            4.0 * (parameters["amp_x"] - parameters["amp_y"])
            + 2.0 * np.pi * parameters["amp_y"]
        )
        / 0.1,
        "straight_line": 2.0 * np.pi * parameters["amp_x"] / 0.1,
    }
    for trajectory_type in ("circle", "racetrack", "straight_line"):
        start = trajectory.trajectory_kinematics(trajectory_type, 0.0)
        ramped = trajectory.trajectory_kinematics(trajectory_type, 10.0)
        repeated = trajectory.trajectory_kinematics(
            trajectory_type, 10.0 + periods[trajectory_type]
        )
        speeds = [
            np.linalg.norm(
                trajectory.trajectory_kinematics(trajectory_type, float(time_s))[1]
            )
            for time_s in np.linspace(10.0, 10.0 + periods[trajectory_type], 1001)
        ]

        assert start[0][2] == 0.0
        assert np.allclose(start[1], np.zeros(3))
        assert ramped[0][2] == 0.0
        assert ramped[1][2] == 0.0
        assert np.allclose(ramped[0], repeated[0])
        assert np.allclose(ramped[1], repeated[1])
        assert max(speeds) <= 0.1 + 1.0e-12
        assert np.isclose(max(speeds), 0.1, atol=1.0e-6)

    circle_start = trajectory.circle_kinematics(0.0)
    racetrack_start = trajectory.racetrack_kinematics(0.0)
    line_start = trajectory.straight_line_kinematics(0.0)
    assert np.allclose(circle_start[0], [0.0, -1.2, 0.0])
    assert np.allclose(racetrack_start[0], [-1.1, -1.2, 0.0])
    assert np.allclose(line_start[0], [-2.3, 0.0, 0.0])
    assert circle_start[3] == 0.0
    assert racetrack_start[3] == 0.0
    assert line_start[3] == 0.0


def test_each_automatic_path_approaches_its_own_deterministic_start_at_half_metre():
    trajectory = object.__new__(TrajectoryCommandNode)
    trajectory.initial_position = None
    trajectory.latest_position = np.asarray([1.0, 1.0, 0.8], dtype=np.float64)
    trajectory.motion_start_position = trajectory.latest_position.copy()
    parameters = {
        "relative_to_initial_pose": False,
        "center_x": 2.71,
        "center_y": 1.865,
        "center_z": 0.5,
        "radius_m": 1.2,
        "amp_x": 2.3,
        "amp_y": 1.2,
        "trajectory_speed_mps": 0.1,
        "trajectory_ramp_s": 10.0,
        "move_duration_s": 15.0,
        "trajectory_type": "circle",
    }
    trajectory.get_parameter = lambda name: SimpleNamespace(value=parameters[name])

    expected_xy = {
        "circle": [2.71, 0.665],
        "racetrack": [1.61, 0.665],
        "straight_line": [0.41, 1.865],
    }
    for trajectory_type, start_xy in expected_xy.items():
        parameters["trajectory_type"] = trajectory_type
        goal = trajectory.sample("move_to_trajectory_start", 15.0)
        path_start = trajectory.sample(trajectory_type, 0.0)
        assert np.allclose(goal.position, [*start_xy, 0.5])
        assert np.allclose(goal.position, path_start.position)
        assert np.allclose(goal.velocity, np.zeros(3), atol=1e-12)


def test_spatial_lissajous_envelope_fits_pool_and_rl_speed_limits():
    trajectory = object.__new__(TrajectoryCommandNode)
    trajectory.initial_position = None
    trajectory.initial_quaternion = None
    trajectory.motion_start_position = np.asarray(
        [2.7, 1.8, 0.70], dtype=np.float64
    )
    trajectory.motion_start_quaternion = trajectory.rpy_quaternion(0.0, 0.0, 0.0)
    trajectory.envelope_rejection_reason = ""
    parameters = {
        "trajectory_type": "spatial_lissajous",
        "control_mode": "rl_policy",
        "relative_to_initial_pose": False,
        "center_x": 2.71,
        "center_y": 1.865,
        "center_z": 0.5,
        "pool_min_xyz": [0.0, 0.0, 0.0],
        "pool_max_xyz": [5.42, 3.73, 1.0],
        "trajectory_limits_configured": True,
        "max_linear_speed_mps": 0.55,
        "max_angular_speed_rps": 0.60,
        "attitude_min_rpy_deg": [-15.0, -15.0, -30.0],
        "attitude_max_rpy_deg": [15.0, 15.0, 30.0],
        "amp_x": 1.5,
        "amp_y": 0.75,
        "amp_z": 0.25,
        "trajectory_speed_mps": 0.1,
        "trajectory_ramp_s": 10.0,
        "move_duration_s": 15.0,
        "hold_before_motion_s": 25.0,
        "publish_rate_hz": 60.0,
        "attitude_mode": "path_tangent",
    }
    trajectory.get_parameter = lambda name: SimpleNamespace(value=parameters[name])

    assert trajectory.validate_scenario_envelope() is True

    parameters["amp_z"] = 0.55
    assert trajectory.validate_scenario_envelope() is False


def test_rl_none_hold_uses_fast_station_bounds_at_measured_start_height():
    trajectory = object.__new__(TrajectoryCommandNode)
    trajectory.initial_position = None
    trajectory.initial_quaternion = None
    trajectory.motion_start_position = np.asarray([2.4, 1.7, 0.72])
    trajectory.motion_start_quaternion = trajectory.rpy_quaternion(0.05, -0.04, 2.5)
    trajectory.latest_angular_velocity = np.asarray([0.02, -0.01, 0.03])
    trajectory.envelope_rejection_reason = ""
    parameters = {
        "trajectory_type": "hold",
        "control_mode": "rl_policy",
        "relative_to_initial_pose": False,
        "center_z": 0.5,
        "pool_min_xyz": [0.0, 0.0, 0.0],
        "pool_max_xyz": [5.42, 3.73, 1.0],
        "trajectory_limits_configured": True,
        "max_linear_speed_mps": 0.55,
        "max_angular_speed_rps": 0.60,
        "attitude_min_rpy_deg": [-15.0, -15.0, -30.0],
        "attitude_max_rpy_deg": [15.0, 15.0, 30.0],
    }
    trajectory.get_parameter = lambda name: SimpleNamespace(value=parameters[name])

    # Match Station Hold Fast: latch current X/Y/Z and heading, then let the
    # right-stick rate update and re-latch measured height.
    assert trajectory.validate_scenario_envelope() is True

    # A stale configured center height is deliberately ignored by these modes.
    parameters["center_z"] = 1.1
    assert trajectory.validate_scenario_envelope() is True

    trajectory.motion_start_position[2] = 1.1
    assert trajectory.validate_scenario_envelope() is False


def test_new_planar_trajectory_envelopes_fit_the_pool_at_half_metre():
    trajectory = object.__new__(TrajectoryCommandNode)
    trajectory.initial_position = None
    trajectory.initial_quaternion = None
    trajectory.motion_start_position = np.asarray(
        [2.7, 1.8, 0.70], dtype=np.float64
    )
    trajectory.motion_start_quaternion = trajectory.rpy_quaternion(0.0, 0.0, 2.4)
    trajectory.envelope_rejection_reason = ""
    parameters = {
        "trajectory_type": "circle",
        "control_mode": "station_hold_fast",
        "relative_to_initial_pose": False,
        "center_x": 2.71,
        "center_y": 1.865,
        "center_z": 0.5,
        "pool_min_xyz": [0.0, 0.0, 0.0],
        "pool_max_xyz": [5.42, 3.73, 1.0],
        "trajectory_limits_configured": True,
        "max_linear_speed_mps": 0.55,
        "max_angular_speed_rps": 0.60,
        "attitude_min_rpy_deg": [-15.0, -15.0, -30.0],
        "attitude_max_rpy_deg": [15.0, 15.0, 30.0],
        "radius_m": 1.2,
        "amp_x": 2.3,
        "amp_y": 1.2,
        "trajectory_speed_mps": 0.1,
        "trajectory_ramp_s": 10.0,
        "move_duration_s": 15.0,
        "hold_before_motion_s": 25.0,
        "publish_rate_hz": 60.0,
        "attitude_mode": "path_tangent",
    }
    trajectory.get_parameter = lambda name: SimpleNamespace(value=parameters[name])

    for trajectory_type in ("circle", "racetrack", "straight_line"):
        parameters["trajectory_type"] = trajectory_type
        assert trajectory.validate_scenario_envelope() is True
        for time_s in np.linspace(0.0, 110.0, 361):
            sample = trajectory.sample(trajectory_type, float(time_s))
            assert sample.position[2] == 0.5


def test_idle_hold_tracks_latest_valid_body_position():
    trajectory = object.__new__(TrajectoryCommandNode)
    trajectory.latest_position = np.asarray([0.3, 1.17, 0.85], dtype=np.float64)

    sample = trajectory.idle_hold_sample()

    assert sample.position == (0.3, 1.17, 0.85)
    assert sample.velocity == (0.0, 0.0, 0.0)
    assert sample.acceleration == (0.0, 0.0, 0.0)


def test_every_station_gamepad_target_uses_fast_four_axis_mapping():
    trajectory = object.__new__(TrajectoryCommandNode)
    trajectory.motion_start_position = np.asarray([1.7, 1.1, 0.9], dtype=np.float64)
    trajectory.motion_start_quaternion = trajectory.rpy_quaternion(0.0, 0.0, 0.3)
    trajectory.latest_position = trajectory.motion_start_position.copy()
    trajectory.latest_quaternion = trajectory.motion_start_quaternion.copy()
    trajectory.station_target_position = trajectory.motion_start_position.copy()
    trajectory.station_target_quaternion = trajectory.motion_start_quaternion.copy()
    trajectory.station_planar_active = False
    trajectory.station_heave_active = False
    trajectory.station_yaw_active = False
    trajectory.operator_target_input = SimpleNamespace(
        twist=SimpleNamespace(
            linear=SimpleNamespace(x=0.8, y=-0.7, z=0.5),
            angular=SimpleNamespace(z=0.4),
        )
    )
    trajectory.operator_target_input_ns = 1_000_000_000
    parameters = {
        "manual_input_age_s": 0.15,
        "manual_vertical_speed_mps": 0.2,
        "station_linear_input_gain_mps": 0.2,
        "station_lateral_input_gain_mps": 0.2,
        "station_yaw_input_gain_rps": 0.3,
        "control_mode": "station_hold",
        "require_pool_bounds": False,
    }
    trajectory.get_parameter = lambda name: SimpleNamespace(value=parameters[name])

    sample, _, angular_velocity = trajectory.station_hold_target(1_020_000_000)

    expected_world = trajectory.rotation_matrix(
        trajectory.latest_quaternion
    ) @ np.asarray([0.16, -0.14, 0.0])
    assert np.allclose(sample.velocity[:2], expected_world[:2])
    assert sample.velocity[2] == -0.1
    assert np.linalg.norm(angular_velocity) > 0.0


def test_fast_station_hold_maps_right_stick_horizontal_to_lateral_velocity():
    trajectory = object.__new__(TrajectoryCommandNode)
    trajectory.motion_start_position = np.asarray([1.7, 1.1, 0.9], dtype=np.float64)
    trajectory.motion_start_quaternion = trajectory.rpy_quaternion(0.0, 0.0, 0.3)
    trajectory.latest_position = trajectory.motion_start_position.copy()
    trajectory.latest_quaternion = trajectory.motion_start_quaternion.copy()
    trajectory.station_target_position = trajectory.motion_start_position.copy()
    trajectory.station_target_quaternion = trajectory.motion_start_quaternion.copy()
    trajectory.station_planar_active = False
    trajectory.station_heave_active = False
    trajectory.station_yaw_active = False
    trajectory.operator_target_input = SimpleNamespace(
        twist=SimpleNamespace(
            linear=SimpleNamespace(x=0.0, y=-0.7, z=0.0),
            angular=SimpleNamespace(z=0.0),
        )
    )
    trajectory.operator_target_input_ns = 1_000_000_000
    parameters = {
        "manual_input_age_s": 0.15,
        "manual_vertical_speed_mps": 0.2,
        "station_linear_input_gain_mps": 0.2,
        "station_lateral_input_gain_mps": 0.2,
        "station_yaw_input_gain_rps": 0.3,
        "control_mode": "station_hold_fast",
        "require_pool_bounds": False,
    }
    trajectory.get_parameter = lambda name: SimpleNamespace(value=parameters[name])

    sample, _, angular_velocity = trajectory.station_hold_target(1_020_000_000)

    expected_world = trajectory.rotation_matrix(
        trajectory.latest_quaternion
    ) @ np.asarray([0.0, -0.14, 0.0])
    assert np.allclose(sample.velocity[:2], expected_world[:2])
    assert np.allclose(sample.position[:2], trajectory.latest_position[:2])
    assert sample.velocity[2] == 0.0
    assert np.allclose(angular_velocity, 0.0)

    trajectory.latest_position = np.asarray([1.82, 1.04, 0.9], dtype=np.float64)
    trajectory.operator_target_input.twist.linear.y = 0.0
    trajectory.operator_target_input_ns = 1_030_000_000
    released, _, _ = trajectory.station_hold_target(1_040_000_000)

    assert np.allclose(released.position[:2], trajectory.latest_position[:2])
    assert np.allclose(released.velocity[:2], 0.0)


def test_rl_hold_and_fast_station_publish_identical_gamepad_targets():
    parameters = {
        "manual_input_age_s": 0.15,
        "manual_vertical_speed_mps": 0.2,
        "station_linear_input_gain_mps": 0.5,
        "station_lateral_input_gain_mps": 0.2,
        "station_yaw_input_gain_rps": 0.6,
        "require_pool_bounds": True,
        "pool_min_xyz": [0.0, 0.0, 0.2],
        "pool_max_xyz": [4.0, 4.0, 1.2],
    }

    def target_for_mode(mode):
        trajectory = object.__new__(TrajectoryCommandNode)
        trajectory.motion_start_position = np.asarray([1.7, 1.1, 0.9])
        trajectory.motion_start_quaternion = trajectory.rpy_quaternion(
            0.0, 0.0, 0.3
        )
        trajectory.latest_position = np.asarray([1.72, 1.08, 0.88])
        trajectory.latest_quaternion = trajectory.rpy_quaternion(
            0.15, -0.12, 0.3
        )
        trajectory.station_target_position = np.asarray([1.7, 1.1, 0.9])
        trajectory.station_target_quaternion = trajectory.rpy_quaternion(
            0.0, 0.0, 0.3
        )
        trajectory.station_planar_active = False
        trajectory.station_heave_active = False
        trajectory.station_yaw_active = False
        trajectory.operator_target_input = SimpleNamespace(
            twist=SimpleNamespace(
                linear=SimpleNamespace(x=0.7, y=-0.6, z=0.4),
                angular=SimpleNamespace(z=-0.5),
            )
        )
        trajectory.operator_target_input_ns = 1_000_000_000
        mode_parameters = {**parameters, "control_mode": mode}
        trajectory.get_parameter = lambda name: SimpleNamespace(
            value=mode_parameters[name]
        )
        return trajectory.station_hold_target(1_020_000_000)

    fast_sample, fast_orientation, fast_angular = target_for_mode(
        "station_hold_fast"
    )
    rl_sample, rl_orientation, rl_angular = target_for_mode("rl_policy")

    assert rl_sample == fast_sample
    assert np.allclose(rl_orientation, fast_orientation)
    assert np.allclose(rl_angular, fast_angular)


def test_station_horizontal_command_uses_yaw_without_measured_tilt():
    trajectory = object.__new__(TrajectoryCommandNode)
    trajectory.motion_start_position = np.asarray([1.7, 1.1, 0.9], dtype=np.float64)
    trajectory.motion_start_quaternion = trajectory.rpy_quaternion(0.0, 0.0, 0.3)
    trajectory.latest_position = trajectory.motion_start_position.copy()
    trajectory.latest_quaternion = trajectory.rpy_quaternion(0.35, -0.25, 0.3)
    trajectory.station_target_position = trajectory.motion_start_position.copy()
    trajectory.station_target_quaternion = trajectory.motion_start_quaternion.copy()
    trajectory.station_planar_active = False
    trajectory.station_heave_active = False
    trajectory.station_yaw_active = False
    trajectory.operator_target_input = SimpleNamespace(
        twist=SimpleNamespace(
            linear=SimpleNamespace(x=1.0, y=0.0, z=0.0),
            angular=SimpleNamespace(z=0.0),
        )
    )
    trajectory.operator_target_input_ns = 1_000_000_000
    parameters = {
        "manual_input_age_s": 0.15,
        "manual_vertical_speed_mps": 0.2,
        "station_linear_input_gain_mps": 0.2,
        "station_lateral_input_gain_mps": 0.2,
        "station_yaw_input_gain_rps": 0.3,
        "control_mode": "station_hold_fast",
        "require_pool_bounds": False,
    }
    trajectory.get_parameter = lambda name: SimpleNamespace(value=parameters[name])

    sample, _, angular_velocity = trajectory.station_hold_target(1_020_000_000)

    assert np.allclose(
        sample.velocity,
        [0.2 * np.cos(0.3), 0.2 * np.sin(0.3), 0.0],
        atol=1e-12,
    )
    assert np.allclose(angular_velocity, 0.0)


def test_station_right_stick_tracks_measured_height_and_latches_on_release():
    trajectory = object.__new__(TrajectoryCommandNode)
    trajectory.motion_start_position = np.asarray([1.7, 1.1, 0.9], dtype=np.float64)
    trajectory.motion_start_quaternion = trajectory.rpy_quaternion(0.0, 0.0, 0.3)
    trajectory.latest_position = trajectory.motion_start_position.copy()
    trajectory.latest_quaternion = trajectory.motion_start_quaternion.copy()
    trajectory.station_target_position = trajectory.motion_start_position.copy()
    trajectory.station_target_quaternion = trajectory.motion_start_quaternion.copy()
    trajectory.station_planar_active = False
    trajectory.station_heave_active = False
    trajectory.station_yaw_active = False
    trajectory.operator_target_input = SimpleNamespace(
        twist=SimpleNamespace(
            linear=SimpleNamespace(x=0.0, y=0.0, z=0.5),
            angular=SimpleNamespace(z=0.0),
        )
    )
    trajectory.operator_target_input_ns = 1_000_000_000
    parameters = {
        "manual_input_age_s": 0.15,
        "manual_vertical_speed_mps": 0.2,
        "station_linear_input_gain_mps": 0.2,
        "station_lateral_input_gain_mps": 0.2,
        "station_yaw_input_gain_rps": 0.3,
        "control_mode": "station_hold_fast",
        "require_pool_bounds": True,
        "pool_min_xyz": [0.0, 0.0, 0.2],
        "pool_max_xyz": [4.0, 4.0, 1.2],
    }
    trajectory.get_parameter = lambda name: SimpleNamespace(value=parameters[name])

    moving, _, _ = trajectory.station_hold_target(1_100_000_000)
    assert np.isclose(moving.position[2], 0.9)
    assert np.isclose(moving.velocity[2], -0.1)

    trajectory.latest_position[2] = 0.86
    trajectory.operator_target_input.twist.linear.z = 0.0
    trajectory.operator_target_input_ns = 1_110_000_000
    held, _, _ = trajectory.station_hold_target(1_120_000_000)
    assert np.isclose(held.position[2], 0.86)
    assert held.velocity[2] == 0.0


def test_station_hold_validation_latches_the_latest_complete_pose():
    trajectory = object.__new__(TrajectoryCommandNode)
    trajectory.latest_position = np.asarray([1.4, 1.8, 0.76], dtype=np.float64)
    trajectory.latest_quaternion = np.asarray(
        [0.96592583, 0.0, 0.0, 0.25881905], dtype=np.float64
    )
    trajectory.motion_start_position = None
    trajectory.motion_start_quaternion = None
    trajectory.envelope_checked = False
    trajectory.envelope_valid = False
    trajectory.get_parameter = lambda name: SimpleNamespace(
        value={
            "trajectory_type": "hold",
            "control_mode": "station_hold",
        }[name]
    )
    trajectory.validate_scenario_envelope = lambda: True
    response = SimpleNamespace(success=False, message="")

    trajectory.on_validate_scenario(None, response)
    trajectory.latest_position[:] = [2.0, 2.2, 0.62]
    trajectory.latest_quaternion[:] = [1.0, 0.0, 0.0, 0.0]

    assert response.success is True
    assert np.allclose(trajectory.motion_start_position, [1.4, 1.8, 0.76])
    assert np.allclose(
        trajectory.motion_start_quaternion,
        [0.96592583, 0.0, 0.0, 0.25881905],
    )


def make_altitude_hold_trajectory():
    trajectory = object.__new__(TrajectoryCommandNode)
    trajectory.latest_position = np.asarray([1.2, 0.8, 0.62], dtype=np.float64)
    trajectory.motion_start_position = trajectory.latest_position.copy()
    trajectory.altitude_hold_z = 0.9
    trajectory.altitude_heave_active = False
    trajectory.manual_command = SimpleNamespace(
        action=[0.5, 0.5, 0.5, 0.5, 0.0, 0.0, 0.0, 0.0],
        enable=True,
        source="web_operator",
    )
    trajectory.manual_command_ns = 1_050_000_000
    parameters = {
        "manual_input_age_s": 0.15,
        "manual_vertical_speed_mps": 0.2,
    }
    trajectory.get_parameter = lambda name: SimpleNamespace(value=parameters[name])
    return trajectory


def test_altitude_hold_uses_configured_initial_height_before_vertical_input():
    trajectory = make_altitude_hold_trajectory()
    trajectory.manual_command.enable = False

    first = trajectory.altitude_hold_sample(1_050_000_000)
    trajectory.latest_position[:] = [1.7, 1.1, 0.58]
    following = trajectory.altitude_hold_sample(1_060_000_000)

    assert first.position == (1.2, 0.8, 0.9)
    assert following.position == (1.7, 1.1, 0.9)
    assert trajectory.altitude_hold_z == 0.9


def test_altitude_hold_stops_and_latches_measured_height_on_stick_release():
    trajectory = make_altitude_hold_trajectory()

    commanded = trajectory.altitude_hold_sample(1_100_000_000)
    trajectory.latest_position[:] = [1.7, 1.1, 0.55]
    trajectory.manual_command.enable = False
    released = trajectory.altitude_hold_sample(1_110_000_000)
    trajectory.latest_position[2] = 0.50
    held = trajectory.altitude_hold_sample(1_120_000_000)

    assert commanded.position[2] == 0.62
    assert commanded.velocity == (0.0, 0.0, -0.1)
    assert released.position[2] == 0.55
    assert released.velocity == (0.0, 0.0, 0.0)
    assert held.position[2] == 0.55
    assert trajectory.altitude_hold_z == 0.55
    assert held.velocity == (0.0, 0.0, 0.0)
