"""Regression checks for the CUDA detector and mapped-localisation split."""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
LAUNCH = ROOT / "ros_ws" / "src" / "eup_bringup" / "launch" / "eup_edge_system.launch.py"
LOCALIZER = (
    ROOT
    / "ros_ws"
    / "src"
    / "eup_sensors"
    / "eup_sensors"
    / "apriltag_localization_node.py"
)


def test_production_detector_is_isaac_ros_cuda():
    launch = LAUNCH.read_text(encoding="utf-8")

    assert 'package="isaac_ros_apriltag"' in launch
    assert 'plugin="nvidia::isaac_ros::apriltag::AprilTagNode"' in launch
    assert '"backends": "CUDA"' in launch
    assert '"tag_family": "tag36h11"' in launch
    assert '("tf", "/localization/apriltag/raw_tf")' in launch
    assert 'package="eup_apriltag_vpi"' not in launch
    assert not (ROOT / "ros_ws" / "src" / "eup_apriltag_vpi").exists()


def test_map_localizer_consumes_cuda_corners_without_detecting_images():
    source = LOCALIZER.read_text(encoding="utf-8")

    assert "from isaac_ros_apriltag_interfaces.msg import AprilTagDetectionArray" in source
    assert '"detections_topic", "/localization/apriltag/detections"' in source
    assert "def on_detections(" in source
    assert "isaac_ros_tag36h11_corners_in_map_axis_order" in source
    assert "detectMarkers" not in source
    assert "ArucoDetector" not in source
    assert "vpiSubmitAprilTagDetector" not in source


def test_map_is_loaded_only_at_startup_or_by_explicit_relocalization():
    source = LOCALIZER.read_text(encoding="utf-8")

    assert "tag_map_reload_interval_s" not in source
    assert "reload_tag_layout_if_changed" not in source
    assert "tag_map_mtime_ns" not in source
    assert "def on_relocalize(" in source


def test_weak_tags_are_removed_without_rejecting_strong_tags_or_logging_acceptance():
    source = LOCALIZER.read_text(encoding="utf-8")

    assert "filter_tag_correspondences_by_minimum_edge(" in source
    assert "continuing with {seen_ids}" in source
    assert "degraded two-Tag VIO validation accepted" not in source


def test_isaac_raw_pose_is_not_used_for_mixed_tag_sizes():
    launch = LAUNCH.read_text(encoding="utf-8")
    source = LOCALIZER.read_text(encoding="utf-8")

    assert '"size": 0.4' in launch
    assert "detection.pose" not in source
    assert 'definition["size_m"]' in source
