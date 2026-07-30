"""Pure SE(3) helpers for AprilTag calibration of ZED VIO odometry."""

from __future__ import annotations

from dataclasses import dataclass, field
import math

import numpy as np

from .localization_math import (
    invert_transform,
    quaternion_xyzw,
    rotation_from_quaternion_xyzw,
    slerp_quaternion_xyzw,
)


def map_from_odom_from_tag(map_from_base: np.ndarray, odom_from_base: np.ndarray) -> np.ndarray:
    """Calibrate the local VIO origin from one map-frame Tag observation."""

    return np.asarray(map_from_base, dtype=np.float64) @ invert_transform(
        np.asarray(odom_from_base, dtype=np.float64)
    )


def map_from_base_from_vio(map_from_odom: np.ndarray, odom_from_base: np.ndarray) -> np.ndarray:
    """Express the current ZED VIO pose in the global AprilTag map frame."""

    return np.asarray(map_from_odom, dtype=np.float64) @ np.asarray(
        odom_from_base, dtype=np.float64
    )


def interpolate_transform(before: np.ndarray, after: np.ndarray, fraction: float) -> np.ndarray:
    """Interpolate two timestamp-bracketing rigid transforms."""

    before = np.asarray(before, dtype=np.float64)
    after = np.asarray(after, dtype=np.float64)
    if before.shape != (4, 4) or after.shape != (4, 4):
        raise ValueError("both transforms must be 4x4 matrices")
    fraction = min(1.0, max(0.0, float(fraction)))
    result = np.eye(4, dtype=np.float64)
    result[:3, :3] = rotation_from_quaternion_xyzw(
        slerp_quaternion_xyzw(
            quaternion_xyzw(before[:3, :3]),
            quaternion_xyzw(after[:3, :3]),
            fraction,
        )
    )
    result[:3, 3] = (1.0 - fraction) * before[:3, 3] + fraction * after[:3, 3]
    return result


def blend_transform(previous: np.ndarray, observed: np.ndarray, alpha: float) -> np.ndarray:
    """Blend a bounded alignment correction without interpolating matrix entries."""

    fraction = min(1.0, max(0.0, float(alpha)))
    previous = np.asarray(previous, dtype=np.float64)
    observed = np.asarray(observed, dtype=np.float64)
    result = np.eye(4, dtype=np.float64)
    result[:3, :3] = rotation_from_quaternion_xyzw(
        slerp_quaternion_xyzw(
            quaternion_xyzw(previous[:3, :3]),
            quaternion_xyzw(observed[:3, :3]),
            fraction,
        )
    )
    result[:3, 3] = (1.0 - fraction) * previous[:3, 3] + fraction * observed[:3, 3]
    return result


def rotation_distance_rad(first: np.ndarray, second: np.ndarray) -> float:
    """Return the smallest angular distance between two rigid rotations."""

    relative = np.asarray(first, dtype=np.float64)[:3, :3].T @ np.asarray(
        second, dtype=np.float64
    )[:3, :3]
    cosine = (float(np.trace(relative)) - 1.0) / 2.0
    return math.acos(min(1.0, max(-1.0, cosine)))


def representative_transform(transforms: list[np.ndarray]) -> np.ndarray:
    """Average an already-consistent group of rigid transforms.

    Translation is averaged arithmetically.  Unit quaternions use the same
    hemisphere as the first sample before averaging, so equivalent ``q`` and
    ``-q`` rotations cannot cancel each other out.
    """

    if not transforms:
        raise ValueError("at least one transform is required")
    samples = [np.asarray(transform, dtype=np.float64) for transform in transforms]
    if any(sample.shape != (4, 4) for sample in samples):
        raise ValueError("each transform must be a 4x4 matrix")

    quaternions = [
        np.asarray(quaternion_xyzw(sample[:3, :3]), dtype=np.float64) for sample in samples
    ]
    reference = quaternions[0]
    aligned = [
        quaternion if float(np.dot(quaternion, reference)) >= 0.0 else -quaternion
        for quaternion in quaternions
    ]
    mean_quaternion = np.sum(aligned, axis=0)
    norm = float(np.linalg.norm(mean_quaternion))
    if not math.isfinite(norm) or norm <= 1e-12:
        raise ValueError("cannot average degenerate rotations")

    result = np.eye(4, dtype=np.float64)
    result[:3, :3] = rotation_from_quaternion_xyzw(mean_quaternion / norm)
    result[:3, 3] = np.mean([sample[:3, 3] for sample in samples], axis=0)
    return result


@dataclass
class AlignmentCandidateWindow:
    """Collect a short, mutually-consistent Tag/VIO map-alignment window."""

    transforms: list[np.ndarray] = field(default_factory=list)
    first_stamp_ns: int = 0
    last_stamp_ns: int = 0

    def add(
        self,
        observed: np.ndarray,
        stamp_ns: int,
        *,
        confirm_frames: int,
        max_spread_m: float,
        max_angle_rad: float,
        max_gap_ns: int,
    ) -> bool:
        """Add a candidate and report whether its confirmation window is full."""

        observed = np.asarray(observed, dtype=np.float64).copy()
        if observed.shape != (4, 4):
            raise ValueError("observed alignment must be a 4x4 matrix")
        reference = self.transforms[0] if self.transforms else None
        consistent = (
            reference is not None
            and stamp_ns > self.last_stamp_ns
            and (max_gap_ns == 0 or stamp_ns - self.last_stamp_ns <= max_gap_ns)
            and (max_gap_ns == 0 or stamp_ns - self.first_stamp_ns <= max_gap_ns)
            and float(np.linalg.norm(observed[:3, 3] - reference[:3, 3])) <= max_spread_m
            and rotation_distance_rad(reference, observed) <= max_angle_rad
        )
        if not consistent:
            self.transforms = [observed]
            self.first_stamp_ns = stamp_ns
        else:
            self.transforms.append(observed)
        self.last_stamp_ns = stamp_ns
        return len(self.transforms) >= max(1, int(confirm_frames))

    def representative(self) -> np.ndarray:
        """Return the average alignment for a confirmed candidate window."""

        return representative_transform(self.transforms)

    def clear(self):
        self.transforms.clear()
        self.first_stamp_ns = 0
        self.last_stamp_ns = 0
