import math

import numpy as np

from scripts.analyze_tracking_runs import (
    estimate_step_settling_time,
    estimate_trajectory_delay,
    plot_run,
    quaternion_wxyz_to_rpy,
    select_experiment_window,
)


def test_quaternion_to_rpy_is_sign_invariant():
    quaternion = np.asarray([[math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5)]])
    positive = quaternion_wxyz_to_rpy(quaternion)
    negative = quaternion_wxyz_to_rpy(-quaternion)
    np.testing.assert_allclose(positive, negative, atol=1e-12)


def test_trajectory_delay_uses_dominant_pose_axis():
    time_s = np.arange(200, dtype=np.float64) * 0.05
    target = np.zeros((200, 6), dtype=np.float64)
    target[:, 0] = np.sin(2.0 * math.pi * time_s / 3.0)
    actual = np.zeros_like(target)
    actual[4:, 0] = target[:-4, 0]
    delay = estimate_trajectory_delay(time_s, target, actual)
    assert 0.15 <= delay <= 0.25


def test_step_settling_time_requires_remaining_samples_in_band():
    time_s = np.arange(10, dtype=np.float64)
    target = np.zeros((10, 6), dtype=np.float64)
    target[2:, 0] = 0.10
    actual = np.zeros_like(target)
    actual[2:, 0] = [0.02, 0.06, 0.085, 0.095, 0.101, 0.099, 0.10, 0.10]
    assert estimate_step_settling_time(time_s, target, actual, "step_x") == 2.0


def test_experiment_window_requires_an_explicit_start_marker():
    import pytest

    with pytest.raises(ValueError, match="tracking_experiment start marker"):
        select_experiment_window([{"type": "tracking_status", "time": 1}])


def test_plot_run_emits_all_required_figures(tmp_path):
    tracking = []
    for index in range(12):
        tracking.append(
            {
                "payload": {
                    "time_s": index * 0.1,
                    "target_position": [index * 0.01, 0.0, 0.0],
                    "actual_position": [index * 0.009, 0.0, 0.0],
                    "target_orientation_wxyz": [1.0, 0.0, 0.0, 0.0],
                    "actual_orientation_wxyz": [1.0, 0.0, 0.0, 0.0],
                    "position_error_m": index * 0.001,
                    "orientation_error_rad": 0.0,
                    "target_velocity_body": [0.1, 0.0, 0.0],
                    "actual_velocity_body": [0.09, 0.0, 0.0],
                    "target_angular_velocity_body": [0.0, 0.0, 0.0],
                    "actual_angular_velocity_body": [0.0, 0.0, 0.0],
                }
            }
        )
    plot_run("smoke", tracking, np.zeros((12, 8)), tmp_path)
    expected = {
        "smoke_tracking.png",
        "smoke_six_axis_pose.png",
        "smoke_six_axis_error.png",
        "smoke_six_axis_velocity.png",
        "smoke_thrusters.png",
    }
    assert expected == {path.name for path in tmp_path.glob("*.png")}
