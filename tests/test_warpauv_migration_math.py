"""Tests for Isaac AUV math migrated into EUP."""

import math
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros_ws/eup_mujoco_env"))
sys.path.insert(0, str(ROOT / "ros_ws/src/eup_control"))

from eup_mujoco_env.hydrodynamics import (  # noqa: E402
    HydrodynamicConfig,
    HydrodynamicForceModel,
)
from eup_control.warpauv_thruster_model import (  # noqa: E402
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


def test_hydrodynamic_damping_dissipates_relative_motion():
    model = HydrodynamicForceModel()
    nu_r = (0.2, -0.1, 0.3, 0.04, -0.02, 0.01)
    damping = model.calculate_relative_damping_wrench(nu_r)
    assert sum(nu_r[index] * damping[index] for index in range(6)) <= 0.0


def test_buoyancy_matches_isaac_warpauv_constants():
    model = HydrodynamicForceModel()
    force_b, torque_b = model.calculate_buoyancy_forces((1.0, 0.0, 0.0, 0.0))
    expected_z = 997.0 * 0.022747843530591776 * 9.81
    assert math.isclose(force_b[2], expected_z, rel_tol=0.0, abs_tol=1.0e-9)
    assert math.isclose(torque_b[0], 0.0, rel_tol=0.0, abs_tol=1.0e-9)
    assert math.isclose(torque_b[1], 0.0, rel_tol=0.0, abs_tol=1.0e-9)


def test_added_mass_coriolis_preserves_power():
    cfg = HydrodynamicConfig(added_mass_diag=(1.0, 1.2, 1.4, 0.2, 0.25, 0.3))
    model = HydrodynamicForceModel(config=cfg)
    nu_r = (0.3, -0.2, 0.1, 0.04, -0.05, 0.02)
    c_nu = model.calculate_added_mass_coriolis_wrench(nu_r)
    power = sum(nu_r[index] * c_nu[index] for index in range(6))
    assert math.isclose(power, 0.0, rel_tol=0.0, abs_tol=1.0e-12)
