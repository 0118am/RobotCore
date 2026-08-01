"""Pure geometry helpers for AprilTag-based map localisation.

Transforms use the ROS convention ``T_parent_child``: points expressed in the
child frame are transformed into the parent frame by left multiplication.
"""

from __future__ import annotations

import math
from typing import Iterable

import numpy as np


def largest_tag_quads(corners, ids, valid_ids: Iterable[int]) -> list[tuple[int, np.ndarray]]:
    """Return one best-sampled image quad for each valid decoded tag ID."""

    if ids is None:
        return []
    valid = {int(tag_id) for tag_id in valid_ids}
    best_by_id: dict[int, tuple[float, np.ndarray]] = {}
    for raw_id, detected_corners in zip(np.asarray(ids).reshape(-1), corners):
        tag_id = int(raw_id)
        if tag_id not in valid:
            continue
        points = np.asarray(detected_corners, dtype=np.float64).reshape(4, 2)
        x = points[:, 0]
        y = points[:, 1]
        area_px2 = 0.5 * abs(
            float(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1)))
        )
        previous = best_by_id.get(tag_id)
        if previous is None or area_px2 > previous[0]:
            best_by_id[tag_id] = (area_px2, points)
    return [(tag_id, best_by_id[tag_id][1]) for tag_id in sorted(best_by_id)]


def tag_ids_with_inlier_corner_count(
    inliers, seen_ids: list[int], minimum_corners: int
) -> list[int]:
    """Return Tag IDs supported by at least ``minimum_corners`` RANSAC corners."""

    required = min(4, max(1, int(minimum_corners)))
    counts = np.zeros(len(seen_ids), dtype=np.intp)
    for raw_index in np.asarray(inliers, dtype=np.intp).reshape(-1):
        corner_index = int(raw_index)
        if 0 <= corner_index < 4 * len(seen_ids):
            counts[corner_index // 4] += 1
    return [
        int(tag_id)
        for tag_id, count in zip(seen_ids, counts)
        if int(count) >= required
    ]


def per_tag_full_corner_rms_px(residuals, seen_ids: list[int]) -> dict[int, float]:
    """Return each Tag's RMS using all four ordered corners, never only inliers."""

    points = np.asarray(residuals, dtype=np.float64)
    if points.shape != (4 * len(seen_ids), 2):
        raise ValueError("residuals must contain exactly four 2D corners per Tag")
    per_tag = points.reshape(-1, 4, 2)
    return {
        int(tag_id): float(math.sqrt(np.mean(np.sum(tag_points * tag_points, axis=1))))
        for tag_id, tag_points in zip(seen_ids, per_tag)
    }


def filter_tag_correspondences_by_minimum_edge(
    object_points,
    image_points,
    seen_ids: list[int],
    minimum_edge_px: float,
) -> tuple[np.ndarray, np.ndarray, list[int], list[int]]:
    """Remove undersampled Tags without discarding stronger Tags in the frame."""

    objects = np.asarray(object_points, dtype=np.float64)
    images = np.asarray(image_points, dtype=np.float64)
    tag_count = len(seen_ids)
    if objects.shape != (4 * tag_count, 3):
        raise ValueError("object_points must contain exactly four 3D corners per Tag")
    if images.shape != (4 * tag_count, 2):
        raise ValueError("image_points must contain exactly four 2D corners per Tag")

    object_tags = objects.reshape(tag_count, 4, 3)
    image_tags = images.reshape(tag_count, 4, 2)
    threshold_px = max(0.0, float(minimum_edge_px))
    minimum_edges_px = np.min(
        np.linalg.norm(image_tags - np.roll(image_tags, -1, axis=1), axis=2),
        axis=1,
    )
    keep = minimum_edges_px >= threshold_px
    accepted_ids = [int(tag_id) for tag_id, accepted in zip(seen_ids, keep) if accepted]
    rejected_ids = [int(tag_id) for tag_id, accepted in zip(seen_ids, keep) if not accepted]
    return (
        object_tags[keep].reshape(-1, 3),
        image_tags[keep].reshape(-1, 2),
        accepted_ids,
        rejected_ids,
    )


def tag36h11_corners_in_map_axis_order(opencv_corners) -> np.ndarray:
    """Convert OpenCV tag36h11 corners to the physical map's tag-axis order.

    The pool's printed tag36h11 assets use the AprilRobotics frame: +X is
    printed right and +Y is printed top.  OpenCV's predefined AprilTag
    dictionary resolves the same upright printed asset with a canonical frame
    rotated 180 degrees, returning physical bottom-right, bottom-left,
    top-left, top-right.  Rotate the sequence by two corners so it matches the
    physical top-left, top-right, bottom-right, bottom-left order used by
    :func:`tag_corners_in_map`.
    """

    points = np.asarray(opencv_corners, dtype=np.float64).reshape(4, 2)
    return np.roll(points, -2, axis=0)


def isaac_ros_tag36h11_corners_in_map_axis_order(isaac_corners) -> np.ndarray:
    """Return CUDA detector corners in the physical printed-axis order.

    ``isaac_ros_apriltag`` normalizes its CUDA and VPI implementations to the
    AprilTag message convention before publishing: top-left, top-right,
    bottom-right, bottom-left in the decoded tag frame. That is already the
    order used by :func:`tag_corners_in_map`; unlike OpenCV's predefined
    dictionary output, no 180-degree rotation is required.
    """

    points = np.asarray(isaac_corners, dtype=np.float64).reshape(4, 2)
    if not np.isfinite(points).all():
        raise ValueError("Isaac ROS AprilTag corners must be finite")
    return points.copy()


def rpy_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Return the active ROS fixed-axis Rz(yaw) * Ry(pitch) * Rx(roll) rotation."""

    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=np.float64,
    )


def transform_matrix(translation: Iterable[float], rpy: Iterable[float]) -> np.ndarray:
    """Build a homogeneous transform from metres and radians."""

    translation = np.asarray(list(translation), dtype=np.float64)
    rpy = np.asarray(list(rpy), dtype=np.float64)
    if translation.shape != (3,) or rpy.shape != (3,):
        raise ValueError("translation and rpy must each contain exactly three values")
    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = rpy_matrix(*rpy)
    matrix[:3, 3] = translation
    return matrix


def invert_transform(matrix: np.ndarray) -> np.ndarray:
    """Invert a rigid homogeneous transform."""

    rotation = np.asarray(matrix, dtype=np.float64)[:3, :3]
    translation = np.asarray(matrix, dtype=np.float64)[:3, 3]
    inverse = np.eye(4, dtype=np.float64)
    inverse[:3, :3] = rotation.T
    inverse[:3, 3] = -rotation.T @ translation
    return inverse


def tag_corners_in_map(
    center_m: Iterable[float], rpy_rad: Iterable[float], tag_size_m: float
) -> np.ndarray:
    """Return physical AprilTag corners in map-axis order.

    The configured tag frame is ROS right-handed: +X is the printed tag's
    right, +Y is its top, and +Z is normal out of the printed face into the
    water. Corners are returned as physical top-left, top-right, bottom-right,
    bottom-left. Detector output must first be normalized with the helper for
    its source convention.
    """

    size = float(tag_size_m)
    if not math.isfinite(size) or size <= 0.0:
        raise ValueError("tag_size_m must be a positive finite value")
    half = size / 2.0
    tag_corners = np.array(
        [
            [-half, half, 0.0],
            [half, half, 0.0],
            [half, -half, 0.0],
            [-half, -half, 0.0],
        ],
        dtype=np.float64,
    )
    transform = transform_matrix(center_m, rpy_rad)
    return (transform[:3, :3] @ tag_corners.T).T + transform[:3, 3]


def validate_cuboid_pool_tag_layout(
    layout: dict[int, dict[str, np.ndarray]],
    *,
    pool_length_m: float,
    pool_width_m: float,
    surface_tolerance_m: float = 0.02,
    orientation_tolerance_deg: float = 2.0,
) -> dict[str, list[int]]:
    """Validate that every mapped Tag lies correctly on the pool floor or an inner wall.

    The map origin is the lower-right-rear pool corner. +F spans
    ``pool_length_m``, +L spans ``pool_width_m``, and +U points upward.
    Wall-tag +Y must point upward and every tag-face +Z must point into the
    pool. Floor tags may use any in-plane yaw, but their +Z must point upward.
    """

    length_m = float(pool_length_m)
    width_m = float(pool_width_m)
    surface_tolerance_m = max(0.0, float(surface_tolerance_m))
    orientation_tolerance_deg = max(0.0, float(orientation_tolerance_deg))
    if (
        not math.isfinite(length_m)
        or not math.isfinite(width_m)
        or length_m <= 0.0
        or width_m <= 0.0
    ):
        raise ValueError("pool_length_m and pool_width_m must be positive finite values")
    if not math.isfinite(surface_tolerance_m):
        raise ValueError("surface_tolerance_m must be finite")
    if not math.isfinite(orientation_tolerance_deg):
        raise ValueError("orientation_tolerance_deg must be finite")

    minimum_axis_dot = math.cos(math.radians(min(180.0, orientation_tolerance_deg)))
    expected_surfaces = {
        "floor": (2, 0.0, np.asarray([0.0, 0.0, 1.0])),
        "rear": (0, 0.0, np.asarray([1.0, 0.0, 0.0])),
        "front": (0, length_m, np.asarray([-1.0, 0.0, 0.0])),
        "right": (1, 0.0, np.asarray([0.0, 1.0, 0.0])),
        "left": (1, width_m, np.asarray([0.0, -1.0, 0.0])),
    }
    groups: dict[str, list[int]] = {name: [] for name in expected_surfaces}
    upward = np.asarray([0.0, 0.0, 1.0])

    for tag_id, definition in sorted(layout.items()):
        center = np.asarray(definition["position_m"], dtype=np.float64)
        rpy = np.asarray(definition["rpy_rad"], dtype=np.float64)
        size_m = float(definition["size_m"])
        rotation = rpy_matrix(*rpy)
        face_normal = rotation[:, 2]
        matching_surfaces = [
            name
            for name, (axis, coordinate, inward_normal) in expected_surfaces.items()
            if abs(float(center[axis]) - coordinate) <= surface_tolerance_m
            and float(np.dot(face_normal, inward_normal)) >= minimum_axis_dot
        ]
        if len(matching_surfaces) != 1:
            raise ValueError(
                f"tag {tag_id} does not have one unambiguous inward-facing pool surface"
            )
        surface = matching_surfaces[0]
        if surface != "floor" and float(np.dot(rotation[:, 1], upward)) < minimum_axis_dot:
            raise ValueError(f"wall tag {tag_id} printed top does not point toward +U")

        corners = tag_corners_in_map(center, rpy, size_m)
        if (
            float(np.min(corners[:, 0])) < -surface_tolerance_m
            or float(np.max(corners[:, 0])) > length_m + surface_tolerance_m
            or float(np.min(corners[:, 1])) < -surface_tolerance_m
            or float(np.max(corners[:, 1])) > width_m + surface_tolerance_m
            or float(np.min(corners[:, 2])) < -surface_tolerance_m
        ):
            raise ValueError(f"tag {tag_id} corners extend outside the pool bounds")
        groups[surface].append(int(tag_id))

    return groups


def quaternion_xyzw(rotation: np.ndarray) -> tuple[float, float, float, float]:
    """Convert a 3x3 rotation matrix to a normalized ROS xyzw quaternion."""

    matrix = np.asarray(rotation, dtype=np.float64)
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * scale
        x = (matrix[2, 1] - matrix[1, 2]) / scale
        y = (matrix[0, 2] - matrix[2, 0]) / scale
        z = (matrix[1, 0] - matrix[0, 1]) / scale
    elif matrix[0, 0] > matrix[1, 1] and matrix[0, 0] > matrix[2, 2]:
        scale = math.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
        w = (matrix[2, 1] - matrix[1, 2]) / scale
        x = 0.25 * scale
        y = (matrix[0, 1] + matrix[1, 0]) / scale
        z = (matrix[0, 2] + matrix[2, 0]) / scale
    elif matrix[1, 1] > matrix[2, 2]:
        scale = math.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
        w = (matrix[0, 2] - matrix[2, 0]) / scale
        x = (matrix[0, 1] + matrix[1, 0]) / scale
        y = 0.25 * scale
        z = (matrix[1, 2] + matrix[2, 1]) / scale
    else:
        scale = math.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
        w = (matrix[1, 0] - matrix[0, 1]) / scale
        x = (matrix[0, 2] + matrix[2, 0]) / scale
        y = (matrix[1, 2] + matrix[2, 1]) / scale
        z = 0.25 * scale
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    return x / norm, y / norm, z / norm, w / norm


def rotation_from_quaternion_xyzw(quaternion: Iterable[float]) -> np.ndarray:
    """Return a 3x3 rotation matrix from a normalized ROS xyzw quaternion."""

    x, y, z, w = np.asarray(list(quaternion), dtype=np.float64)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(norm) or norm <= 1e-12:
        raise ValueError("quaternion must have a finite non-zero norm")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def slerp_quaternion_xyzw(start: Iterable[float], end: Iterable[float], fraction: float) -> np.ndarray:
    """Interpolate two ROS xyzw quaternions along the shortest rotation."""

    first = np.asarray(list(start), dtype=np.float64)
    second = np.asarray(list(end), dtype=np.float64)
    if first.shape != (4,) or second.shape != (4,):
        raise ValueError("start and end must each contain exactly four values")
    first /= np.linalg.norm(first)
    second /= np.linalg.norm(second)
    dot = float(np.dot(first, second))
    if dot < 0.0:
        second = -second
        dot = -dot
    alpha = min(1.0, max(0.0, float(fraction)))
    if dot > 0.9995:
        result = first + alpha * (second - first)
        return result / np.linalg.norm(result)
    theta = math.acos(min(1.0, max(-1.0, dot)))
    sine = math.sin(theta)
    result = (math.sin((1.0 - alpha) * theta) / sine) * first + (math.sin(alpha * theta) / sine) * second
    return result / np.linalg.norm(result)
