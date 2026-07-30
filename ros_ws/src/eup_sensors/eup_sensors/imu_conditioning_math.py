"""Pure helpers for stationary calibration of the UART8 external IMU."""

from __future__ import annotations

import math
from typing import Iterable

import numpy as np


STANDARD_GRAVITY_MPS2 = 9.80665


def vector3(values: Iterable[float]) -> np.ndarray:
    vector = np.asarray(list(values), dtype=np.float64)
    if vector.shape != (3,) or not np.isfinite(vector).all():
        raise ValueError("expected three finite values")
    return vector


def is_stationary(
    acceleration_mps2: Iterable[float],
    angular_velocity_rps: Iterable[float],
    *,
    gravity_mps2: float = STANDARD_GRAVITY_MPS2,
    acceleration_tolerance_mps2: float = 1.0,
    angular_velocity_limit_rps: float = 0.04,
) -> bool:
    """Return whether a raw specific-force/gyro sample is stationary."""

    acceleration = vector3(acceleration_mps2)
    angular_velocity = vector3(angular_velocity_rps)
    gravity = float(gravity_mps2)
    if not math.isfinite(gravity) or gravity <= 0.0:
        raise ValueError("gravity_mps2 must be positive and finite")
    return (
        abs(float(np.linalg.norm(acceleration)) - gravity)
        <= max(0.0, float(acceleration_tolerance_mps2))
        and float(np.linalg.norm(angular_velocity))
        <= max(0.0, float(angular_velocity_limit_rps))
    )


def gyro_calibration(
    samples: Iterable[Iterable[float]], *, noise_floor_rps: float
) -> tuple[np.ndarray, np.ndarray]:
    """Return stationary gyro bias and conservative per-axis standard deviation."""

    values = np.asarray(list(samples), dtype=np.float64)
    if (
        values.ndim != 2
        or values.shape[0] == 0
        or values.shape[1] != 3
        or not np.isfinite(values).all()
    ):
        raise ValueError("gyro samples must be a non-empty finite Nx3 array")
    bias = np.mean(values, axis=0)
    standard_deviation = np.maximum(
        np.std(values, axis=0), max(0.0, float(noise_floor_rps))
    )
    return bias, standard_deviation


def message_period_is_usable(previous_ns: int | None, current_ns: int, maximum_gap_s: float) -> bool:
    """Reject duplicate/backward samples and gaps that cannot be integrated safely."""

    if previous_ns is None:
        return True
    delta_ns = int(current_ns) - int(previous_ns)
    return 0 < delta_ns <= max(0, int(float(maximum_gap_s) * 1e9))
