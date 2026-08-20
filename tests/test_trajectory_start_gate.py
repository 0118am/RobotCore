"""Trajectory time must remain gated until the operator presses Start."""

import inspect
from types import SimpleNamespace

import numpy as np

from robotcore_runtime.trajectory_command_node import TrajectoryCommandNode


def test_idle_target_is_explicitly_neutral_until_start():
    source = inspect.getsource(TrajectoryCommandNode.tick)

    assert 'published_trajectory_type = "idle"' in source


def make_trajectory():
    trajectory = object.__new__(TrajectoryCommandNode)
    trajectory.started_ns = 1_000_000_000
    trajectory.tracking_started = False
    trajectory.envelope_checked = True
    trajectory.envelope_valid = True
    trajectory.motion_start_position = None
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


def test_idle_hold_tracks_latest_valid_body_position():
    trajectory = object.__new__(TrajectoryCommandNode)
    trajectory.latest_position = np.asarray([0.3, 1.17, 0.85], dtype=np.float64)

    sample = trajectory.idle_hold_sample()

    assert sample.position == (0.3, 1.17, 0.85)
    assert sample.velocity == (0.0, 0.0, 0.0)
    assert sample.acceleration == (0.0, 0.0, 0.0)


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
