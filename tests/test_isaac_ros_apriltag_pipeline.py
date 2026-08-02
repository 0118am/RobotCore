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
    assert '"pose_status_topic", "/localization/apriltag/pose_status"' in source
    assert "status.reprojection_rms_px" in source
    assert "status.rejection_reason" in source
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
    assert "AprilTag correction accepted from mapped Tags" not in source


def test_isaac_raw_pose_is_not_used_for_mixed_tag_sizes():
    launch = LAUNCH.read_text(encoding="utf-8")
    source = LOCALIZER.read_text(encoding="utf-8")

    assert '"size": 0.4' in launch
    assert "detection.pose" not in source
    assert 'definition.get("size_m", default_size)' in source
    assert '"corners_m": tag_corners_in_map(center, rpy, size)' in source


def test_detector_capacity_is_bounded_and_degraded_validation_uses_direct_vio():
    launch = LAUNCH.read_text(encoding="utf-8")
    cpp = (ROOT / "ros_ws/src/eup_sensors/src/apriltag_map_localizer_component.cpp").read_text()

    assert 'DeclareLaunchArgument("apriltag_max_tags", default_value="24")' in launch
    assert 'plugin="eup_sensors::AprilTagMapLocalizerComponent"' in launch
    assert '"aligned_vio_odometry_topic", "/localization/aligned_vio_odom"' in cpp


def test_zed_nitros_is_enabled_while_operator_video_remains_on_demand():
    launch = LAUNCH.read_text(encoding="utf-8")
    camera_config = (
        ROOT / "ros_ws" / "src" / "eup_sensors" / "config" / "zedx_minimal_open.yaml"
    ).read_text(encoding="utf-8")
    zed_launcher = (
        ROOT / "ros_ws" / "src" / "eup_runtime" / "eup_runtime" / "zed_camera_launcher.py"
    ).read_text(encoding="utf-8")

    assert 'plugin="nvidia::isaac_ros::image_proc::ImageFormatConverterNode"' in launch
    assert '("image_raw", LaunchConfiguration("front_camera_raw_topic"))' in launch
    assert '("image", "/localization/apriltag/image_rgb")' in launch
    assert '"encoding_desired": "rgb8"' in launch
    assert '"image_raw_nitros_format": "nitros_image_bgr8"' in launch
    assert '"image_nitros_format": "nitros_image_rgb8"' in launch
    assert "front_camera_nitros_topic" not in launch
    assert "disable_nitros: false" in camera_config
    assert '"enable_ipc:=false"' in zed_launcher


def test_imu_latched_status_does_not_disable_high_rate_intra_process_path():
    conditioner = (
        ROOT / "ros_ws/src/eup_sensors/src/imu_conditioner_component.cpp"
    ).read_text(encoding="utf-8")

    assert "status_options.use_intra_process_comm" in conditioner
    assert "rclcpp::IntraProcessSetting::Disable" in conditioner
