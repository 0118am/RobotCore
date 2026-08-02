"""Behavioral tests for localisation freshness and time alignment."""

from collections import deque
from types import SimpleNamespace

import numpy as np

from eup_interfaces.msg import AprilTagPoseStatus, LocalizationStatus
from eup_sensors.apriltag_localization_node import AprilTagLocalizationNode
from eup_sensors.localization_math import transform_matrix
from eup_sensors.sensor_fusion_node import SensorFusionNode


def test_message_freshness_requires_both_recent_source_and_arrival_time():
    now_ns = 10_000_000_000
    check = SensorFusionNode.message_is_fresh
    assert check(
        None,
        arrival_ns=now_ns - 50_000_000,
        source_stamp_ns=now_ns - 100_000_000,
        now_ns=now_ns,
        maximum_age_s=0.2,
    )
    assert not check(
        None,
        arrival_ns=now_ns - 50_000_000,
        source_stamp_ns=now_ns - 300_000_000,
        now_ns=now_ns,
        maximum_age_s=0.2,
    )
    assert not check(
        None,
        arrival_ns=now_ns - 300_000_000,
        source_stamp_ns=now_ns - 50_000_000,
        now_ns=now_ns,
        maximum_age_s=0.2,
    )


def test_fused_pose_alignment_interpolates_but_never_extrapolates():
    fake = SimpleNamespace(
        fused_history=deque(
            [
                (1_000_000_000, transform_matrix([0, 0, 0], [0, 0, 0])),
                (1_040_000_000, transform_matrix([2, 0, 0], [0, 0, 0])),
            ]
        ),
        get_parameter=lambda _name: SimpleNamespace(value=0.05),
    )

    interpolated = SensorFusionNode.fused_pose_at(fake, 1_020_000_000)
    np.testing.assert_allclose(interpolated[:3, 3], [1.0, 0.0, 0.0])
    assert SensorFusionNode.fused_pose_at(fake, 990_000_000) is None
    assert SensorFusionNode.fused_pose_at(fake, 1_050_000_000) is None


def test_reported_rate_falls_to_zero_after_stream_stops():
    fake = SimpleNamespace(
        get_parameter=lambda _name: SimpleNamespace(value=1.0),
    )
    samples = deque([1_000_000_000, 1_100_000_000, 1_200_000_000])
    assert SensorFusionNode.rate_hz(fake, samples, 1_200_000_000) == 10.0
    assert SensorFusionNode.rate_hz(fake, samples, 2_500_000_000) == 0.0


def test_localization_status_has_fixed_covariance_storage():
    status = LocalizationStatus()
    assert len(status.pose_covariance) == 36
    assert len(status.twist_covariance) == 36
    tag_status = AprilTagPoseStatus()
    assert not tag_status.pose_published


class FakeProjector:
    def __init__(self, projected):
        self.projected = np.asarray(projected, dtype=np.float64)

    def projectPoints(self, *_args):
        return self.projected.reshape(-1, 1, 2), None


def tag_quads(edge_px):
    return np.asarray(
        [
            [[offset, 0], [offset + edge_px, 0], [offset + edge_px, edge_px], [offset, edge_px]]
            for offset in (0, 100, 200)
        ],
        dtype=np.float64,
    ).reshape(-1, 2)


def uncertainty_scale(image_points, projected):
    fake = SimpleNamespace(
        cv2=FakeProjector(projected),
        camera_matrix=np.eye(3),
        distortion=np.zeros(5),
        runtime_parameters={
            "max_reprojection_rms_px": 3.0,
            "min_tag_edge_px": 20.0,
            "minimum_pose_tag_count": 3,
        },
    )
    return AprilTagLocalizationNode.pose_uncertainty_scale(
        fake,
        object_points=np.zeros((12, 3)),
        image_points=image_points,
        seen_ids=[1, 2, 3],
        inliers=np.arange(12).reshape(-1, 1),
        rvec=np.zeros(3),
        tvec=np.zeros(3),
        degraded=False,
    )


def test_tag_covariance_scale_responds_to_reprojection_and_apparent_size():
    strong = tag_quads(80.0)
    weak = tag_quads(20.0)
    assert uncertainty_scale(strong, strong) == 0.5
    assert uncertainty_scale(weak, weak - [3.0, 0.0]) > 2.0
