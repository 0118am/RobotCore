"""Pure control mathematics for the six degree-of-freedom PID controller.

The module deliberately has no ROS imports so quaternion and anti-windup
behaviour can be tested on development machines without a sourced ROS install.
Quaternions use the ``(w, x, y, z)`` ordering used by the policy code.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable

import numpy as np


def vec(values: Iterable[float], size: int) -> np.ndarray:
    result = np.asarray(list(values), dtype=np.float64).reshape(-1)
    if result.size != size or not np.all(np.isfinite(result)):
        raise ValueError(f"expected {size} finite values")
    return result


def altitude_velocity_setpoint(
    target_height: float,
    actual_height: float,
    target_vertical_velocity: float,
    position_kp: float,
) -> float:
    """Convert map-frame height error into an unrestricted speed target."""

    values = np.asarray(
        [
            target_height,
            actual_height,
            target_vertical_velocity,
            position_kp,
        ],
        dtype=np.float64,
    )
    if not np.all(np.isfinite(values)):
        raise ValueError("altitude-loop inputs must be finite")
    return float(target_vertical_velocity) + float(position_kp) * (
        float(target_height) - float(actual_height)
    )


def altitude_collective_pwm_commands(
    pid_output: float, command_limit: float, command_sign: float
) -> np.ndarray:
    """Apply one direct PID effort to the four vertical PWM channels."""

    if not all(
        math.isfinite(value) for value in (pid_output, command_limit, command_sign)
    ):
        raise ValueError("altitude PWM inputs must be finite")
    if math.isclose(command_sign, 0.0, abs_tol=1e-12):
        raise ValueError("altitude PWM command sign must be non-zero")
    limit = abs(float(command_limit))
    common = float(
        np.clip(math.copysign(1.0, command_sign) * pid_output, -limit, limit)
    )
    commands = np.zeros(8, dtype=np.float64)
    commands[:4] = common
    return commands


def first_order_low_pass(
    previous: float | None,
    sample: float,
    dt: float,
    time_constant_s: float,
) -> float:
    """Filter one finite sample without adding an actuator command ramp."""

    values = (sample, dt, time_constant_s)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("low-pass inputs must be finite")
    if dt <= 0.0:
        raise ValueError("low-pass dt must be positive")
    if previous is None or time_constant_s <= 0.0:
        return float(sample)
    if not math.isfinite(previous):
        raise ValueError("low-pass previous value must be finite")
    alpha = -math.expm1(-dt / time_constant_s)
    return float(previous + alpha * (sample - previous))


def manual_surge_yaw_commands(commands: Iterable[float]) -> np.ndarray:
    """Keep only manual surge/yaw and return their T5--T8 command pattern.

    Browser/manual candidates are physical thruster vectors.  Projecting onto
    the two approved mixer basis vectors prevents vertical or roll input from
    leaking into altitude-hold mode, including from an older browser client.
    """

    values = vec(commands, 8)
    upper = values[4:]
    surge = 0.25 * (-upper[0] - upper[1] + upper[2] + upper[3])
    yaw = 0.25 * (-upper[0] + upper[1] + upper[2] - upper[3])
    result = np.zeros(8, dtype=np.float64)
    result[4:] = [
        -surge - yaw,
        -surge + yaw,
        surge + yaw,
        surge - yaw,
    ]
    return np.clip(result, -1.0, 1.0)


def normalize_quaternion(quaternion: Iterable[float]) -> np.ndarray:
    q = vec(quaternion, 4)
    norm = float(np.linalg.norm(q))
    if norm <= 1e-9:
        raise ValueError("quaternion norm is zero")
    return q / norm


def quaternion_conjugate(quaternion: Iterable[float]) -> np.ndarray:
    q = normalize_quaternion(quaternion)
    return np.array([q[0], -q[1], -q[2], -q[3]], dtype=np.float64)


def quaternion_multiply(left: Iterable[float], right: Iterable[float]) -> np.ndarray:
    lw, lx, ly, lz = normalize_quaternion(left)
    rw, rx, ry, rz = normalize_quaternion(right)
    return normalize_quaternion(
        [
            lw * rw - lx * rx - ly * ry - lz * rz,
            lw * rx + lx * rw + ly * rz - lz * ry,
            lw * ry - lx * rz + ly * rw + lz * rx,
            lw * rz + lx * ry - ly * rx + lz * rw,
        ]
    )


def quaternion_apply(quaternion: Iterable[float], vector: Iterable[float]) -> np.ndarray:
    q = normalize_quaternion(quaternion)
    value = vec(vector, 3)
    xyz = q[1:4]
    twice_cross = 2.0 * np.cross(xyz, value)
    return value + q[0] * twice_cross + np.cross(xyz, twice_cross)


def quaternion_error_body(
    current_world_from_body: Iterable[float],
    target_world_from_body: Iterable[float],
) -> np.ndarray:
    """Return the shortest target rotation as a body-frame rotation vector."""

    relative = quaternion_multiply(
        quaternion_conjugate(current_world_from_body),
        target_world_from_body,
    )
    if relative[0] < 0.0:
        relative = -relative
    vector_norm = float(np.linalg.norm(relative[1:4]))
    if vector_norm <= 1e-10:
        return 2.0 * relative[1:4]
    angle = 2.0 * math.atan2(vector_norm, max(0.0, float(relative[0])))
    return relative[1:4] * (angle / vector_norm)


def rpy_to_quaternion(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Convert fixed-axis roll/pitch/yaw radians to a normalized quaternion."""

    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return normalize_quaternion(
        [
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
        ]
    )


def quaternion_to_rotation_matrix(quaternion: Iterable[float]) -> np.ndarray:
    w, x, y, z = normalize_quaternion(quaternion)
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * w), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


@dataclass(frozen=True)
class PidGains:
    kp: float
    ki: float
    kd: float
    integral_limit: float
    output_limit: float
    derivative_cutoff_hz: float = 5.0


class ConditionalPid:
    """PID with derivative-on-measurement and conditional integration."""

    def __init__(self, gains: PidGains):
        self.gains = gains
        self.integral = 0.0
        self.previous_measurement: float | None = None
        self.filtered_derivative = 0.0
        self.saturated = False

    def reset(self):
        self.integral = 0.0
        self.previous_measurement = None
        self.filtered_derivative = 0.0
        self.saturated = False

    def step(
        self,
        *,
        setpoint: float,
        measurement: float,
        dt: float,
        feedforward: float = 0.0,
        output_limit: float | None = None,
    ) -> float:
        if not all(math.isfinite(value) for value in (setpoint, measurement, dt, feedforward)):
            raise ValueError("PID inputs must be finite")
        if dt <= 0.0:
            raise ValueError("PID dt must be positive")

        error = float(setpoint - measurement)
        raw_derivative = 0.0
        if self.previous_measurement is not None:
            raw_derivative = -(measurement - self.previous_measurement) / dt
        self.previous_measurement = float(measurement)

        cutoff = max(0.0, float(self.gains.derivative_cutoff_hz))
        alpha = 1.0 if cutoff <= 0.0 else 1.0 - math.exp(-2.0 * math.pi * cutoff * dt)
        self.filtered_derivative += alpha * (raw_derivative - self.filtered_derivative)

        proposed_integral = float(
            np.clip(
                self.integral + error * dt,
                -abs(self.gains.integral_limit),
                abs(self.gains.integral_limit),
            )
        )
        unsaturated = (
            feedforward
            + self.gains.kp * error
            + self.gains.ki * proposed_integral
            + self.gains.kd * self.filtered_derivative
        )
        limit = abs(
            float(self.gains.output_limit)
            if output_limit is None
            else float(output_limit)
        )
        if not math.isfinite(limit):
            raise ValueError("PID output limit must be finite")
        saturated = float(np.clip(unsaturated, -limit, limit))

        # Integrate when unsaturated, or when the error would pull an already
        # saturated command back toward the feasible interval.
        can_integrate = (
            math.isclose(unsaturated, saturated, rel_tol=0.0, abs_tol=1e-12)
            or (unsaturated > limit and error < 0.0)
            or (unsaturated < -limit and error > 0.0)
        )
        if can_integrate:
            self.integral = proposed_integral
        else:
            unsaturated = (
                feedforward
                + self.gains.kp * error
                + self.gains.ki * self.integral
                + self.gains.kd * self.filtered_derivative
            )
            saturated = float(np.clip(unsaturated, -limit, limit))

        self.saturated = not math.isclose(unsaturated, saturated, rel_tol=0.0, abs_tol=1e-12)
        return saturated


class SixAxisPid:
    """Six independent inner-loop PID axes with a shared reset operation."""

    def __init__(self, gains: Iterable[PidGains]):
        gain_list = list(gains)
        if len(gain_list) != 6:
            raise ValueError("six PID gain sets are required")
        self.axes = [ConditionalPid(item) for item in gain_list]

    def reset(self):
        for axis in self.axes:
            axis.reset()

    def step(self, setpoint, measurement, dt: float, feedforward=None) -> np.ndarray:
        desired = vec(setpoint, 6)
        actual = vec(measurement, 6)
        ff = np.zeros(6, dtype=np.float64) if feedforward is None else vec(feedforward, 6)
        return np.asarray(
            [
                axis.step(
                    setpoint=float(desired[index]),
                    measurement=float(actual[index]),
                    dt=dt,
                    feedforward=float(ff[index]),
                )
                for index, axis in enumerate(self.axes)
            ],
            dtype=np.float64,
        )
