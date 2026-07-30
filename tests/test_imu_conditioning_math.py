"""Tests for UART8 external-IMU stationary gyro calibration."""

import importlib.util
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = ROOT / "ros_ws/src/eup_sensors/eup_sensors/imu_conditioning_math.py"
SPEC = importlib.util.spec_from_file_location("imu_conditioning_math", MODULE_PATH)
MATH = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MATH)


def test_stationary_accepts_specific_force_and_small_gyro():
    assert MATH.is_stationary(
        [0.1, -0.1, MATH.STANDARD_GRAVITY_MPS2 + 0.2],
        [0.001, -0.002, 0.003],
    )


def test_stationary_rejects_motion_and_gravity_removed_input():
    assert not MATH.is_stationary(
        [0.0, 0.0, MATH.STANDARD_GRAVITY_MPS2],
        [0.0, 0.0, 0.2],
    )
    assert not MATH.is_stationary([0.1, -0.1, 0.2], [0.0, 0.0, 0.0])


def test_gyro_calibration_returns_bias_and_noise_floor():
    bias, stddev = MATH.gyro_calibration(
        [[0.01, -0.02, 0.03]] * 5,
        noise_floor_rps=0.005,
    )
    np.testing.assert_allclose(bias, [0.01, -0.02, 0.03])
    np.testing.assert_allclose(stddev, [0.005, 0.005, 0.005])


def test_message_period_rejects_duplicate_backward_and_large_gap():
    assert MATH.message_period_is_usable(None, 100, 0.2)
    assert MATH.message_period_is_usable(100, 200, 0.2)
    assert not MATH.message_period_is_usable(100, 100, 0.2)
    assert not MATH.message_period_is_usable(200, 100, 0.2)
    assert not MATH.message_period_is_usable(0, 300_000_000, 0.2)
