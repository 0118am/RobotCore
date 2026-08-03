"""Tests for global AprilTag calibration of local ZED VIO odometry."""

import sys
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros_ws/src/robotcore_sensors"))

from robotcore_sensors.localization_math import transform_matrix
from robotcore_sensors.tag_vio_alignment_math import (  # noqa: E402
    AlignmentCandidateWindow,
    aligned_pose_covariance,
    blend_transform,
    interpolate_transform,
    map_from_base_from_vio,
    map_from_odom_from_tag,
)


def test_tag_observation_establishes_global_vio_alignment():
    odom_from_base = transform_matrix([1.2, -0.5, 0.1], [0.0, 0.0, 0.2])
    map_from_base = transform_matrix([4.0, 1.5, 0.6], [0.0, 0.0, -0.4])

    map_from_odom = map_from_odom_from_tag(map_from_base, odom_from_base)

    np.testing.assert_allclose(
        map_from_base_from_vio(map_from_odom, odom_from_base), map_from_base, atol=1e-12
    )


def test_alignment_waits_for_web_relocalize_after_startup():
    source = (
        ROOT / "ros_ws/src/robotcore_sensors/robotcore_sensors/tag_vio_alignment_node.py"
    ).read_text(encoding="utf-8")

    assert "self.relocalize_requested = False" in source
    assert "self.relocalize_requested = True" in source
    assert "if not self.relocalize_requested:" in source
    assert "use AprilTag map Relocalize to establish map->odom" in source


def test_relocalization_replaces_any_old_alignment_without_a_jump_limit():
    source = (
        ROOT / "ros_ws/src/robotcore_sensors/robotcore_sensors/tag_vio_alignment_node.py"
    ).read_text(encoding="utf-8")

    assert "self.relocalization_pending = True" in source
    relocalized_branch = source.index("if self.relocalization_pending:")
    correction_limit_branch = source.index("alignment_max_correction_m", relocalized_branch)
    assert relocalized_branch < correction_limit_branch
    assert "self.map_from_odom = confirmed_alignment" in source[
        relocalized_branch:correction_limit_branch
    ]
    assert "self.relocalization_pending = False" in source[
        relocalized_branch:correction_limit_branch
    ]


def test_alignment_keeps_global_pose_continuous_during_tag_loss():
    map_from_odom = transform_matrix([3.0, -1.0, 0.4], [0.0, 0.0, 0.3])
    first_vio = transform_matrix([0.5, 0.2, 0.0], [0.0, 0.0, 0.1])
    later_vio = transform_matrix([1.1, 0.5, -0.1], [0.0, 0.0, 0.2])

    first_global = map_from_base_from_vio(map_from_odom, first_vio)
    later_global = map_from_base_from_vio(map_from_odom, later_vio)

    expected_delta = map_from_odom[:3, :3] @ (later_vio[:3, 3] - first_vio[:3, 3])
    np.testing.assert_allclose(later_global[:3, 3] - first_global[:3, 3], expected_delta, atol=1e-12)


def test_alignment_correction_blends_translation_and_rotation():
    previous = transform_matrix([0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
    observed = transform_matrix([2.0, 0.0, 0.0], [0.0, 0.0, np.pi])

    blended = blend_transform(previous, observed, 0.5)

    np.testing.assert_allclose(blended[:3, 3], [1.0, 0.0, 0.0], atol=1e-12)
    np.testing.assert_allclose(blended[:3, :3] @ [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], atol=1e-12)


def test_vio_transform_interpolation_matches_tag_timestamp():
    before = transform_matrix([0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
    after = transform_matrix([2.0, 4.0, 0.0], [0.0, 0.0, np.pi])

    interpolated = interpolate_transform(before, after, 0.5)

    np.testing.assert_allclose(interpolated[:3, 3], [1.0, 2.0, 0.0], atol=1e-12)
    np.testing.assert_allclose(interpolated[:3, :3] @ [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], atol=1e-12)


def add_candidate(window, transform, stamp_ns):
    return window.add(
        transform,
        stamp_ns,
        confirm_frames=4,
        max_spread_m=0.20,
        max_angle_rad=np.deg2rad(12.0),
        max_gap_ns=500_000_000,
    )


def test_alignment_candidate_confirms_four_consistent_frames_and_averages_them():
    window = AlignmentCandidateWindow()
    samples = [
        transform_matrix([1.00, 2.0, 0.0], [0.0, 0.0, 0.00]),
        transform_matrix([1.04, 2.0, 0.0], [0.0, 0.0, 0.02]),
        transform_matrix([0.98, 2.0, 0.0], [0.0, 0.0, -0.02]),
        transform_matrix([1.02, 2.0, 0.0], [0.0, 0.0, 0.00]),
    ]

    for index, sample in enumerate(samples[:3]):
        assert not add_candidate(window, sample, 1_000_000_000 + index * 33_000_000)
    assert add_candidate(window, samples[3], 1_099_000_000)

    representative = window.representative()
    np.testing.assert_allclose(representative[:3, 3], [1.01, 2.0, 0.0], atol=1e-12)
    np.testing.assert_allclose(representative[:3, :3], np.eye(3), atol=1e-12)
    covariance = np.asarray(window.covariance()).reshape(6, 6)
    assert covariance[0, 0] > 0.0
    assert covariance[5, 5] > 0.0
    np.testing.assert_allclose(covariance, covariance.T, atol=1e-12)


def test_aligned_pose_covariance_rotates_vio_axes_and_preserves_full_matrix():
    alignment = np.diag([0.01, 0.02, 0.03, 0.04, 0.05, 0.06])
    vio = np.diag([1.0, 2.0, 3.0, 4.0, 5.0, 6.0])
    map_from_odom = transform_matrix([0.0, 0.0, 0.0], [0.0, 0.0, np.pi / 2.0])

    combined = np.asarray(
        aligned_pose_covariance(alignment.reshape(-1), vio.reshape(-1), map_from_odom)
    ).reshape(6, 6)

    np.testing.assert_allclose(
        np.diag(combined),
        [2.01, 1.02, 3.03, 5.04, 4.05, 6.06],
        atol=1e-12,
    )
    np.testing.assert_allclose(combined, combined.T, atol=1e-12)


def test_alignment_candidate_resets_for_time_spread_position_and_angle_outliers():
    for outlier in (
        (transform_matrix([1.0, 0.0, 0.0], [0.0, 0.0, 0.0]), 1_600_000_000),
        (transform_matrix([1.21, 0.0, 0.0], [0.0, 0.0, 0.0]), 1_033_000_000),
        (transform_matrix([1.0, 0.0, 0.0], [0.0, 0.0, np.deg2rad(13.0)]), 1_033_000_000),
    ):
        window = AlignmentCandidateWindow()
        assert not add_candidate(
            window, transform_matrix([1.0, 0.0, 0.0], [0.0, 0.0, 0.0]), 1_000_000_000
        )
        assert not add_candidate(window, *outlier)
        assert len(window.transforms) == 1
