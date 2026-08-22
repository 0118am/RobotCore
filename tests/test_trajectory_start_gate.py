"""Trajectory time must remain gated until the operator presses Start."""

import inspect
from types import SimpleNamespace

import numpy as np

from robotcore_runtime.trajectory_command_node import TrajectoryCommandNode


def test_idle_target_is_explicitly_neutral_until_start():
    source = inspect.getsource(TrajectoryCommandNode.tick)

    assert 'published_trajectory_type = "idle"' in source


def test_trajectory_uses_ekf_position_and_direct_imu_attitude():
    source = inspect.getsource(TrajectoryCommandNode)
    body_callback = inspect.getsource(TrajectoryCommandNode.on_body_state)
    imu_callback = inspect.getsource(TrajectoryCommandNode.on_imu)

    assert 'self.declare_parameter("imu_topic", "/sensors/external_imu")' in source
    assert "qos_profile_sensor_data" in source
    assert "msg.pose.orientation" not in body_callback
    assert "self.latest_position = position" in body_callback
    assert "msg.orientation.w" in imu_callback
    assert "self.latest_quaternion = quaternion / norm" in imu_callback


def make_trajectory():
    trajectory = object.__new__(TrajectoryCommandNode)
    trajectory.started_ns = 1_000_000_000
    trajectory.tracking_started = False
    trajectory.envelope_checked = True
    trajectory.envelope_valid = True
    trajectory.motion_start_position = None
    trajectory.motion_start_quaternion = None
    trajectory.get_parameter = lambda name: SimpleNamespace(
        value={"trajectory_type": "hold"}[name]
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
    trajectory.altitude_hold_z = None
    trajectory.altitude_heave_active = False
    trajectory.get_parameter = lambda name: SimpleNamespace(
        value={"trajectory_type": "altitude_hold", "center_z": 0.9}[name]
    )
    response = SimpleNamespace(success=False, message="")

    trajectory.on_reset_scenario(None, response)

    assert response.success is True
    assert trajectory.altitude_hold_z == 0.9


def test_station_modes_reset_to_level_roll_pitch_and_keep_heading():
    for trajectory_type in ("station_hold", "station_hold_fast"):
        trajectory = make_trajectory()
        trajectory.motion_start_position = np.asarray([1.2, 0.8, 0.62])
        trajectory.motion_start_quaternion = trajectory.rpy_quaternion(
            0.2, -0.1, 0.7
        )
        trajectory.station_target_position = None
        trajectory.station_target_quaternion = None
        trajectory.get_parameter = lambda name, mode=trajectory_type: SimpleNamespace(
            value={"trajectory_type": mode, "center_z": 0.9}[name]
        )
        response = SimpleNamespace(success=False, message="")

        trajectory.on_reset_scenario(None, response)

        target_rpy = trajectory.quaternion_to_rpy(
            trajectory.station_target_quaternion
        )
        assert response.success is True
        assert trajectory.station_target_position[2] == 0.9
        assert np.allclose(target_rpy, [0.0, 0.0, 0.7])


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

    attitude = trajectory.attitude_quaternion("altitude_hold", 0.0)

    assert np.allclose(attitude, trajectory.motion_start_quaternion)


def make_altitude_envelope_trajectory():
    trajectory = object.__new__(TrajectoryCommandNode)
    trajectory.motion_start_position = np.asarray([2.4, 1.1, 0.88], dtype=np.float64)
    trajectory.motion_start_quaternion = trajectory.rpy_quaternion(0.1, -0.1, 2.4)
    trajectory.latest_angular_velocity = np.asarray([0.02, -0.01, 0.04])
    trajectory.initial_position = trajectory.motion_start_position.copy()
    parameters = {
        "trajectory_type": "altitude_hold",
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


def test_spatial_figure_eight_starts_smoothly_and_repeats_in_three_axes():
    trajectory = object.__new__(TrajectoryCommandNode)
    trajectory.initial_position = None
    trajectory.motion_start_position = np.asarray(
        [2.4, 1.7, 0.72], dtype=np.float64
    )
    parameters = {
        "relative_to_initial_pose": True,
        "amp_x": 0.8,
        "amp_y": 0.5,
        "amp_z": 0.2,
        "period_s": 42.0,
        "trajectory_ramp_s": 5.0,
        "publish_rate_hz": 60.0,
    }
    trajectory.get_parameter = lambda name: SimpleNamespace(value=parameters[name])

    start = trajectory.sample("spatial_figure_eight", 0.0)
    ramped = trajectory.sample("spatial_figure_eight", 5.0)
    repeated = trajectory.sample("spatial_figure_eight", 47.0)

    assert np.allclose(start.position, trajectory.motion_start_position)
    assert np.allclose(start.velocity, np.zeros(3))
    assert np.allclose(start.acceleration, np.zeros(3))
    assert not np.isclose(ramped.position[2], trajectory.motion_start_position[2])
    assert np.linalg.norm(ramped.velocity) > 0.0
    assert np.allclose(ramped.position, repeated.position)
    assert np.allclose(ramped.velocity, repeated.velocity)
    start_attitude = trajectory.attitude_quaternion("spatial_figure_eight", 0.0)
    _attitude, angular_velocity, _angular_acceleration = trajectory.sample_attitude(
        "spatial_figure_eight", 5.0
    )
    expected_start_yaw = np.arctan2(2.0 * parameters["amp_y"], parameters["amp_x"])
    assert np.allclose(
        trajectory.quaternion_to_rpy(start_attitude),
        [0.0, 0.0, expected_start_yaw],
    )
    assert abs(angular_velocity[2]) > 0.0


def test_figure_eight_prelude_moves_to_center_then_turns_in_place():
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
        "period_s": 42.0,
        "trajectory_ramp_s": 5.0,
        "move_duration_s": 15.0,
        "hold_before_motion_s": 25.0,
        "publish_rate_hz": 60.0,
    }
    trajectory.get_parameter = lambda name: SimpleNamespace(value=parameters[name])
    tick_source = inspect.getsource(TrajectoryCommandNode.tick)

    turn_duration = (
        parameters["hold_before_motion_s"] - parameters["move_duration_s"]
    )
    move_start = trajectory.sample("move_to_hold", 0.0)
    move_midpoint = trajectory.sample("move_to_hold", 7.5)
    move_end = trajectory.sample("move_to_hold", 15.0)
    attitude_start = trajectory.attitude_quaternion(
        "move_to_figure_eight_start", 0.0
    )
    attitude_end = trajectory.attitude_quaternion(
        "move_to_figure_eight_start", turn_duration
    )
    turn_rates = [
        np.linalg.norm(
            trajectory.angular_velocity_at(
                "move_to_figure_eight_start", time_s, 1.0 / 60.0
            )
        )
        for time_s in np.linspace(0.0, turn_duration, 121)
    ]
    path_start = trajectory.sample("spatial_figure_eight", 0.0)
    path_start_attitude = trajectory.attitude_quaternion(
        "spatial_figure_eight", 0.0
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
    assert 'elif time_s < hold_s:' in tick_source
    assert 'move_time_s = min(time_s, move_s)' in tick_source
    assert 'turn_time_s = max(0.0, time_s - move_s)' in tick_source
    assert 'self.sample("move_to_hold", move_time_s)' in tick_source
    assert '"move_to_figure_eight_start", turn_time_s' in tick_source


def test_figure_eight_left_right_symmetry_axis_points_map_forward():
    trajectory = object.__new__(TrajectoryCommandNode)
    parameters = {
        "amp_x": 0.8,
        "amp_y": 0.5,
        "amp_z": 0.2,
        "period_s": 60.0,
        "trajectory_ramp_s": 0.0,
    }
    trajectory.get_parameter = lambda name: SimpleNamespace(value=parameters[name])

    start = trajectory.spatial_figure_eight_kinematics(0.0)
    forward_lobe = trajectory.spatial_figure_eight_kinematics(15.0)
    rear_lobe = trajectory.spatial_figure_eight_kinematics(45.0)

    assert np.allclose(start[0][:2], [0.0, 0.0])
    assert np.isclose(start[3], np.arctan2(1.0, 0.8))
    assert np.allclose(forward_lobe[0][:2], [0.8, 0.0], atol=1e-12)
    assert np.allclose(rear_lobe[0][:2], [-0.8, 0.0], atol=1e-12)


def test_spatial_figure_eight_envelope_checks_fixed_center_hold_and_path():
    trajectory = object.__new__(TrajectoryCommandNode)
    trajectory.initial_position = None
    trajectory.initial_quaternion = None
    trajectory.motion_start_position = np.asarray(
        [2.7, 1.8, 0.70], dtype=np.float64
    )
    trajectory.motion_start_quaternion = trajectory.rpy_quaternion(0.0, 0.0, 2.4)
    trajectory.envelope_rejection_reason = ""
    parameters = {
        "trajectory_type": "spatial_figure_eight",
        "relative_to_initial_pose": False,
        "center_x": 2.71,
        "center_y": 1.865,
        "center_z": 0.5,
        "pool_min_xyz": [0.0, 0.0, 0.0],
        "pool_max_xyz": [5.42, 3.73, 1.0],
        "trajectory_limits_configured": True,
        "max_linear_speed_mps": 0.45,
        "max_angular_speed_rps": 0.60,
        "attitude_min_rpy_deg": [-15.0, -15.0, -30.0],
        "attitude_max_rpy_deg": [15.0, 15.0, 30.0],
        "amp_x": 0.8,
        "amp_y": 0.5,
        "amp_z": 0.2,
        "period_s": 42.0,
        "trajectory_ramp_s": 5.0,
        "attitude_period_s": 30.0,
        "step_time_s": 5.0,
        "move_duration_s": 15.0,
        "hold_before_motion_s": 25.0,
        "publish_rate_hz": 60.0,
        "attitude_mode": "path_tangent",
    }
    trajectory.get_parameter = lambda name: SimpleNamespace(value=parameters[name])

    assert trajectory.validate_scenario_envelope() is True

    parameters["center_x"] = 0.2
    assert trajectory.validate_scenario_envelope() is False


def test_idle_hold_tracks_latest_valid_body_position():
    trajectory = object.__new__(TrajectoryCommandNode)
    trajectory.latest_position = np.asarray([0.3, 1.17, 0.85], dtype=np.float64)

    sample = trajectory.idle_hold_sample()

    assert sample.position == (0.3, 1.17, 0.85)
    assert sample.velocity == (0.0, 0.0, 0.0)
    assert sample.acceleration == (0.0, 0.0, 0.0)


def test_simple_station_hold_uses_forward_input_but_ignores_lateral_input():
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
        "station_yaw_input_gain_rps": 0.3,
    }
    trajectory.get_parameter = lambda name: SimpleNamespace(value=parameters[name])

    sample, _, angular_velocity = trajectory.station_hold_target(1_020_000_000)

    expected_world = trajectory.rotation_matrix(
        trajectory.latest_quaternion
    ) @ np.asarray([0.16, 0.0, 0.0])
    assert np.allclose(sample.velocity[:2], expected_world[:2])
    assert sample.velocity[2] == 0.1
    assert np.linalg.norm(angular_velocity) > 0.0


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
        value={"trajectory_type": "station_hold"}[name]
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
        normalized=[0.5, 0.5, 0.5, 0.5, 0.0, 0.0, 0.0, 0.0],
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
