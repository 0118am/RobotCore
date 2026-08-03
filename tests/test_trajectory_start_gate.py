"""Trajectory time must remain gated until the operator presses Start."""

from types import SimpleNamespace

from robotcore_runtime.trajectory_command_node import TrajectoryCommandNode


def make_trajectory():
    trajectory = object.__new__(TrajectoryCommandNode)
    trajectory.started_ns = 1_000_000_000
    trajectory.tracking_started = False
    trajectory.envelope_checked = True
    trajectory.envelope_valid = True
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
