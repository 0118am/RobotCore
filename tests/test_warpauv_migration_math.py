"""Tests for the retained pure thruster math; no deleted simulator backend."""

import math
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros_ws/src/robotcore_control"))

from robotcore_control.warpauv_thruster_model import (  # noqa: E402
    first_order_update,
    motor_speed_to_thrust,
    normalized_pwm_to_motor_speed,
    normalized_pwm_to_thrust,
)


def test_warpauv_pwm_deadband_and_quadratic_mapping():
    assert normalized_pwm_to_motor_speed(0.0) == 0.0
    assert normalized_pwm_to_motor_speed(0.079) == 0.0
    forward = normalized_pwm_to_motor_speed(1.0)
    reverse = normalized_pwm_to_motor_speed(-1.0)
    assert math.isclose(forward, 369.28, rel_tol=0.0, abs_tol=1.0e-9)
    assert math.isclose(reverse, -362.58, rel_tol=0.0, abs_tol=1.0e-9)


def test_warpauv_thrust_and_first_order_lag():
    assert motor_speed_to_thrust(10.0, rotor_constant=0.001) == 0.1
    assert motor_speed_to_thrust(-10.0, rotor_constant=0.001) == -0.1
    assert normalized_pwm_to_thrust(0.0) == 0.0
    updated = first_order_update(previous=0.0, command=1.0, dt_s=0.05, tau_s=0.05)
    assert math.isclose(updated, 1.0 - math.exp(-1.0), rel_tol=0.0, abs_tol=1.0e-9)
