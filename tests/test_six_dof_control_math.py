from pathlib import Path
import sys

import numpy as np


PACKAGE_ROOT = Path(__file__).resolve().parents[1] / "ros_ws" / "src" / "robotcore_control"
sys.path.insert(0, str(PACKAGE_ROOT))

from robotcore_control.control_math import (  # noqa: E402
    ConditionalPid,
    PidGains,
    quaternion_apply,
    quaternion_error_body,
    rpy_to_quaternion,
)


def test_quaternion_error_is_sign_invariant_and_uses_shortest_path():
    current = rpy_to_quaternion(0.0, 0.0, np.deg2rad(179.0))
    target = rpy_to_quaternion(0.0, 0.0, np.deg2rad(-179.0))
    error = quaternion_error_body(current, target)
    negated = quaternion_error_body(-current, -target)
    assert np.allclose(error, negated, atol=1e-10)
    assert np.isclose(abs(error[2]), np.deg2rad(2.0), atol=1e-6)


def test_quaternion_rotation_uses_world_from_body_convention():
    yaw_90 = rpy_to_quaternion(0.0, 0.0, np.pi / 2.0)
    rotated = quaternion_apply(yaw_90, [1.0, 0.0, 0.0])
    assert np.allclose(rotated, [0.0, 1.0, 0.0], atol=1e-7)


def test_conditional_pid_does_not_wind_up_while_saturated():
    pid = ConditionalPid(
        PidGains(
            kp=2.0,
            ki=1.0,
            kd=0.0,
            integral_limit=100.0,
            output_limit=1.0,
        )
    )
    outputs = [
        pid.step(setpoint=10.0, measurement=0.0, dt=0.1)
        for _ in range(100)
    ]
    assert outputs == [1.0] * 100
    assert pid.integral == 0.0

    # Reversing the error must immediately pull the command off the positive
    # stop instead of first unwinding a hidden integral.
    output = pid.step(setpoint=-0.1, measurement=0.0, dt=0.1)
    assert output < 0.0


def test_pid_reset_clears_derivative_and_integral_state():
    pid = ConditionalPid(PidGains(1.0, 1.0, 1.0, 5.0, 10.0))
    pid.step(setpoint=1.0, measurement=0.2, dt=0.1)
    pid.step(setpoint=1.0, measurement=0.4, dt=0.1)
    pid.reset()
    assert pid.integral == 0.0
    assert pid.previous_measurement is None
    assert pid.filtered_derivative == 0.0
