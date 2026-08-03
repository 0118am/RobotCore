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

    def step(self, *, setpoint: float, measurement: float, dt: float, feedforward: float = 0.0) -> float:
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
        limit = abs(float(self.gains.output_limit))
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

