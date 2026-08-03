"""WarpAUV thruster command conversion utilities.

Derived from `/home/jining_yang/isaac-auv-env/thruster_dynamics.py` and the
WarpAUV Isaac environment.  The source project is BSD-3-Clause licensed.  This
module keeps the math independent from IsaacLab so ROS2 control tests,
and future edge-side calibration tools can reuse one conversion path.
"""

from __future__ import annotations

from dataclasses import dataclass


PWM_DEADBAND = 0.08
ROTOR_CONSTANT = 0.001
DEFAULT_TAU_S = 0.05


@dataclass(frozen=True)
class ThrusterExtrinsic:
    """Position and orientation of one WarpAUV thruster in base coordinates."""

    name: str
    position_m: tuple[float, float, float]
    rpy_rad: tuple[float, float, float]


WARP_AUV_THRUSTERS = (
    ThrusterExtrinsic("drive_left", (-0.4127, 0.1506, -0.0889), (0.0, 0.0, 0.0)),
    ThrusterExtrinsic("drive_right", (-0.4127, -0.1506, -0.0889), (0.0, 0.0, 0.0)),
    ThrusterExtrinsic("rear_left", (-0.3030, 0.1461, -0.1587), (0.0, -0.785398, 1.5708)),
    ThrusterExtrinsic("rear_right", (-0.3030, -0.1461, -0.1587), (0.0, -0.785398, -1.5708)),
    ThrusterExtrinsic("front_left", (0.0585, 0.1461, -0.0540), (0.0, 0.785398, 1.5708)),
    ThrusterExtrinsic("front_right", (0.0585, -0.1461, -0.0540), (0.0, 0.785398, -1.5708)),
)


def clamp_normalized(value: float) -> float:
    """Clamp a normalized PWM-like command to the policy/action contract."""

    return max(-1.0, min(1.0, float(value)))


def normalized_pwm_to_motor_speed(command: float) -> float:
    """Convert Isaac normalized PWM action to motor-speed-like value.

    Isaac's policy action represents a PWM command in [-1, 1].  The original
    environment applies a deadband and then uses two fitted quadratic curves for
    forward and reverse thrust.  Keeping this helper separate makes it testable
    and lets later hardware calibration replace just this curve.
    """

    value = clamp_normalized(command)
    if abs(value) < PWM_DEADBAND:
        return 0.0
    if value >= PWM_DEADBAND:
        return -139.0 * value**2 + 500.0 * value + 8.28
    return 161.0 * value**2 + 517.86 * value - 5.72


def motor_speed_to_thrust(speed: float, rotor_constant: float = ROTOR_CONSTANT) -> float:
    """Convert motor-speed-like value into signed thrust magnitude."""

    return float(rotor_constant) * abs(float(speed)) * float(speed)


def normalized_pwm_to_thrust(command: float, rotor_constant: float = ROTOR_CONSTANT) -> float:
    """One-shot conversion from normalized action to signed thrust."""

    return motor_speed_to_thrust(
        normalized_pwm_to_motor_speed(command),
        rotor_constant=rotor_constant,
    )


def first_order_update(previous: float, command: float, dt_s: float, tau_s: float = DEFAULT_TAU_S) -> float:
    """Apply the same first-order actuator lag used in the Isaac environment."""

    if tau_s <= 0.0:
        return float(command)
    if dt_s <= 0.0:
        return float(previous)

    # math.exp is imported lazily so this utility stays tiny at module import
    # time for ROS nodes that only need constants or static layout.
    import math

    alpha = math.exp(-float(dt_s) / float(tau_s))
    return float(previous) * alpha + (1.0 - alpha) * float(command)
