"""Tests for the ROS-frame geometry used by AprilTag localisation."""

import importlib.util
import json
import math
from pathlib import Path

import numpy as np
import pytest


ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = ROOT / "ros_ws/src/eup_sensors/eup_sensors/localization_math.py"
SPEC = importlib.util.spec_from_file_location("localization_math", MODULE_PATH)
MATH = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MATH)


def test_tag_corners_follow_ros_tag_axes_and_size():
    corners = MATH.tag_corners_in_map([1.0, 2.0, 3.0], [0.0, 0.0, 0.0], 0.18)

    np.testing.assert_allclose(
        corners,
        [[0.91, 2.09, 3.0], [1.09, 2.09, 3.0], [1.09, 1.91, 3.0], [0.91, 1.91, 3.0]],
    )


def test_opencv_apriltag_corners_are_normalized_to_physical_printed_axes():
    # Live upright pool tags are returned by OpenCV in physical
    # bottom-right, bottom-left, top-left, top-right order.
    opencv_corners = np.asarray(
        [[755.0, 619.0], [586.0, 618.0], [591.0, 447.0], [758.0, 452.0]]
    )

    normalized = MATH.tag36h11_corners_in_map_axis_order(opencv_corners)

    np.testing.assert_allclose(
        normalized,
        [[591.0, 447.0], [758.0, 452.0], [755.0, 619.0], [586.0, 618.0]],
    )


def test_isaac_ros_apriltag_corners_already_follow_physical_printed_axes():
    physical_corners = np.asarray(
        [[591.0, 447.0], [758.0, 452.0], [755.0, 619.0], [586.0, 618.0]]
    )

    normalized = MATH.isaac_ros_tag36h11_corners_in_map_axis_order(
        physical_corners
    )

    np.testing.assert_allclose(normalized, physical_corners)


def test_left_wall_upright_tag_axes_are_forward_up_and_toward_pool_interior():
    rotation = MATH.rpy_matrix(*np.deg2rad([90.0, 0.0, 0.0]))

    np.testing.assert_allclose(rotation @ [1.0, 0.0, 0.0], [1.0, 0.0, 0.0], atol=1e-12)
    np.testing.assert_allclose(rotation @ [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], atol=1e-12)
    np.testing.assert_allclose(rotation @ [0.0, 0.0, 1.0], [0.0, -1.0, 0.0], atol=1e-12)


def test_live_left_wall_tags_fit_one_pose_after_corner_normalization():
    cv2 = pytest.importorskip("cv2")
    camera_matrix = np.asarray(
        [[839.486, 0.0, 639.754], [0.0, 839.486, 401.094], [0.0, 0.0, 1.0]]
    )
    definitions = [
        ([2.227, 3.73, 0.620], 0.4),
        ([3.174, 3.73, 0.633], 0.2),
        ([1.400, 3.73, 0.586], 0.2),
    ]
    live_opencv_corners = [
        [[755.0, 619.0], [586.0, 618.0], [591.0, 447.0], [758.0, 452.0]],
        [[1100.0, 567.0], [1020.0, 567.0], [1023.0, 486.0], [1099.0, 487.0]],
        [[352.0, 593.0], [260.0, 593.0], [264.0, 505.0], [355.0, 506.0]],
    ]
    object_points = np.concatenate(
        [
            MATH.tag_corners_in_map(position, np.deg2rad([90.0, 0.0, 0.0]), size)
            for position, size in definitions
        ]
    )
    image_points = np.concatenate(
        [
            MATH.tag36h11_corners_in_map_axis_order(corners)
            for corners in live_opencv_corners
        ]
    )

    success, rvec, tvec = cv2.solvePnP(
        object_points,
        image_points,
        camera_matrix,
        np.zeros((5, 1)),
        flags=cv2.SOLVEPNP_ITERATIVE,
    )
    assert success
    projected, _ = cv2.projectPoints(
        object_points, rvec, tvec, camera_matrix, np.zeros((5, 1))
    )
    residuals = image_points - projected.reshape(-1, 2)
    rms_px = math.sqrt(float(np.mean(np.sum(residuals * residuals, axis=1))))

    assert rms_px < 3.0


def deployed_pool_layout():
    path = Path("/etc/robotcore/apriltag_map.json")
    if not path.is_file():
        pytest.skip("deployed AprilTag map is not available")
    document = json.loads(path.read_text(encoding="utf-8"))
    return {
        int(tag_id): {
            "position_m": np.asarray(definition["position_m"], dtype=np.float64),
            "rpy_rad": np.deg2rad(definition["rpy_deg"]),
            "size_m": float(definition["size_m"]),
        }
        for tag_id, definition in document["tags"].items()
    }


def test_all_deployed_tags_match_the_cuboid_pool_geometry():
    groups = MATH.validate_cuboid_pool_tag_layout(
        deployed_pool_layout(),
        pool_length_m=5.42,
        pool_width_m=3.73,
    )

    assert groups == {
        "floor": [1, 3, 5, 7, 9, 11, 13, 53, 54, 55, 56, 57, 58, 59, 63, 64, 66, 69, 70, 74, 78, 79, 80],
        "rear": [16, 18, 68, 77],
        "front": [6, 8, 51, 62],
        "right": [0, 2, 4, 52, 61, 65],
        "left": [10, 12, 14, 60, 67, 76],
    }


def test_pool_geometry_rejects_a_left_wall_tag_that_faces_outward():
    invalid_layout = {
        12: {
            "position_m": np.asarray([2.227, 3.73, 0.62]),
            "rpy_rad": np.deg2rad([90.0, 0.0, 180.0]),
            "size_m": 0.4,
        }
    }

    with pytest.raises(ValueError, match="inward-facing pool surface"):
        MATH.validate_cuboid_pool_tag_layout(
            invalid_layout,
            pool_length_m=5.42,
            pool_width_m=3.73,
        )


def test_largest_tag_quads_deduplicates_ids_by_image_area():
    small_7 = np.asarray([[[0, 0], [2, 0], [2, 2], [0, 2]]], dtype=np.float32)
    large_7 = np.asarray([[[10, 10], [20, 10], [20, 20], [10, 20]]], dtype=np.float32)
    tag_8 = np.asarray([[[30, 30], [40, 30], [40, 40], [30, 40]]], dtype=np.float32)

    quads = MATH.largest_tag_quads(
        [small_7, tag_8, large_7],
        np.asarray([[7], [8], [7]], dtype=np.int32),
        {7, 8},
    )

    assert [tag_id for tag_id, _corners in quads] == [7, 8]
    np.testing.assert_allclose(quads[0][1], large_7.reshape(4, 2))
    np.testing.assert_allclose(quads[1][1], tag_8.reshape(4, 2))


def test_tag_support_requires_multiple_inlier_corners_from_each_tag():
    seen_ids = [10, 51, 67]
    # Tag 10 has all four corners, Tag 51 has three, and Tag 67 has only one.
    inliers = np.asarray([[0], [1], [2], [3], [4], [5], [6], [8]])

    assert MATH.tag_ids_with_inlier_corner_count(inliers, seen_ids, 3) == [10, 51]
    assert MATH.tag_ids_with_inlier_corner_count(inliers, seen_ids, 4) == [10]


def test_full_corner_rms_does_not_hide_a_bad_corner_outside_ransac():
    residuals = np.asarray(
        [
            [1.0, 0.0],
            [0.0, 1.0],
            [-1.0, 0.0],
            [0.0, -1.0],
            [1.0, 0.0],
            [0.0, 1.0],
            [-1.0, 0.0],
            [9.0, 0.0],
        ]
    )

    rms = MATH.per_tag_full_corner_rms_px(residuals, [52, 66])

    assert rms[52] == pytest.approx(1.0)
    assert rms[66] == pytest.approx(math.sqrt(21.0))


def test_small_tag_filter_keeps_strong_correspondences_in_the_same_frame():
    object_points = np.arange(36, dtype=np.float64).reshape(12, 3)
    image_points = np.asarray(
        [
            [0, 0], [40, 0], [40, 40], [0, 40],
            [50, 0], [65, 0], [65, 15], [50, 15],
            [100, 0], [130, 0], [130, 30], [100, 30],
        ],
        dtype=np.float64,
    )

    objects, images, accepted, rejected = (
        MATH.filter_tag_correspondences_by_minimum_edge(
            object_points, image_points, [10, 67, 74], 20.0
        )
    )

    assert accepted == [10, 74]
    assert rejected == [67]
    np.testing.assert_allclose(objects, np.concatenate([object_points[:4], object_points[8:]]))
    np.testing.assert_allclose(images, np.concatenate([image_points[:4], image_points[8:]]))


def test_small_tag_filter_can_return_an_empty_well_shaped_observation():
    objects, images, accepted, rejected = MATH.filter_tag_correspondences_by_minimum_edge(
        np.zeros((4, 3)),
        [[0, 0], [10, 0], [10, 10], [0, 10]],
        [67],
        20.0,
    )

    assert objects.shape == (0, 3)
    assert images.shape == (0, 2)
    assert accepted == []
    assert rejected == [67]


def test_tag0_map_rotation_points_the_print_face_normal_toward_positive_x():
    rotation = MATH.rpy_matrix(*np.deg2rad([90.0, 0.0, 90.0]))

    np.testing.assert_allclose(rotation @ [0.0, 0.0, 1.0], [1.0, 0.0, 0.0], atol=1e-12)


def test_tag0_lower_left_corner_is_the_enu_pool_bottom_origin():
    center = [0.0, 0.09, 0.09]
    rpy = np.deg2rad([90.0, 0.0, 90.0])
    rotation = MATH.rpy_matrix(*rpy)
    # Printed tag lower-left is local [-0.09, -0.09, 0].
    lower_left = rotation @ [-0.09, -0.09, 0.0] + np.asarray(center)
    np.testing.assert_allclose(lower_left, [0.0, 0.0, 0.0], atol=1e-12)


def test_camera_mount_offset_is_removed_when_computing_base_pose():
    # A body at the map origin has the measured optical-centre FLU offset.
    base_from_camera = MATH.transform_matrix(
        [0.236, 0.027, 0.016], [-math.pi / 2.0, 0.0, -math.pi / 2.0]
    )
    map_from_camera = base_from_camera
    map_from_base = map_from_camera @ MATH.invert_transform(base_from_camera)

    np.testing.assert_allclose(map_from_base, np.eye(4), atol=1e-12)


def test_transform_inverse_and_quaternion_are_consistent():
    transform = MATH.transform_matrix([0.3, -0.4, 0.8], [0.1, -0.2, 0.3])
    np.testing.assert_allclose(transform @ MATH.invert_transform(transform), np.eye(4), atol=1e-12)

    x, y, z, w = MATH.quaternion_xyzw(transform[:3, :3])
    assert abs(math.sqrt(x * x + y * y + z * z + w * w) - 1.0) < 1e-12


def test_quaternion_slerp_and_rotation_round_trip():
    identity = np.array([0.0, 0.0, 0.0, 1.0])
    half_turn_z = np.array([0.0, 0.0, 1.0, 0.0])
    halfway = MATH.slerp_quaternion_xyzw(identity, half_turn_z, 0.5)
    rotation = MATH.rotation_from_quaternion_xyzw(halfway)

    np.testing.assert_allclose(rotation @ [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], atol=1e-12)
