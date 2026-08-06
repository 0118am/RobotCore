"""Regression checks for the CUDA detector and mapped-localisation split."""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
LAUNCH = ROOT / "ros_ws" / "src" / "robotcore_bringup" / "launch" / "robotcore_edge_system.launch.py"
LOCALIZER = (
    ROOT
    / "ros_ws"
    / "src"
    / "robotcore_sensors"
    / "src"
    / "apriltag_map_localizer_component.cpp"
)
MAP_PARSER = (
    ROOT / "ros_ws/src/robotcore_sensors/include/robotcore_sensors/apriltag_map.hpp"
)


def test_production_detector_is_isaac_ros_cuda():
    launch = LAUNCH.read_text(encoding="utf-8")

    assert 'package="isaac_ros_apriltag"' in launch
    assert 'plugin="nvidia::isaac_ros::apriltag::AprilTagNode"' in launch
    assert '"backends": "CUDA"' in launch
    assert '"tag_family": "tag36h11"' in launch
    assert '("tf", "/localization/apriltag/raw_tf")' in launch
    assert 'package="robotcore_apriltag_vpi"' not in launch
    assert not (ROOT / "ros_ws" / "src" / "robotcore_apriltag_vpi").exists()


def test_map_localizer_consumes_cuda_corners_without_detecting_images():
    source = LOCALIZER.read_text(encoding="utf-8")

    assert "isaac_ros_apriltag_interfaces/msg/april_tag_detection_array.hpp" in source
    assert '"detections_topic", "/localization/apriltag/detections"' in source
    assert "void on_detections(" in source
    assert "robotcore_interfaces/msg/april_tag_pose_estimate.hpp" in source
    assert "estimate.reprojection_rms_px" in source
    assert "estimate.rejection_reason" in source
    assert "estimate.map_generation = map_generation_" in source
    assert "detection.corners" in source
    assert "sensor_msgs/msg/image" not in source
    assert "detectMarkers" not in source
    assert "ArucoDetector" not in source
    assert "vpiSubmitAprilTagDetector" not in source


def test_map_is_loaded_only_at_startup_or_by_explicit_relocalization():
    source = LOCALIZER.read_text(encoding="utf-8")

    assert "tag_map_reload_interval_s" not in source
    assert "reload_tag_layout_if_changed" not in source
    assert "tag_map_mtime_ns" not in source
    assert "bool load_map()" in source
    assert "void reload(" in source


def test_weak_tags_are_removed_without_rejecting_strong_tags_or_logging_acceptance():
    source = LOCALIZER.read_text(encoding="utf-8")

    assert "tag_min_edge < min_edge_px_" in source
    assert "best_area" in source
    assert "degraded two-Tag VIO validation accepted" not in source
    assert "AprilTag correction accepted from mapped Tags" not in source


def test_isaac_raw_pose_is_not_used_for_mixed_tag_sizes():
    launch = LAUNCH.read_text(encoding="utf-8")
    source = LOCALIZER.read_text(encoding="utf-8")
    parser = MAP_PARSER.read_text(encoding="utf-8")

    assert '"size": 0.4' in launch
    assert "detection.pose" not in source
    assert 'contains("size_m")' in parser
    assert 'at("size_m").get<double>()' in parser
    assert "definition.corners[index]" in parser


def test_empty_or_invalid_map_is_fail_closed_without_retaining_deleted_tags():
    source = LOCALIZER.read_text(encoding="utf-8")
    parser = MAP_PARSER.read_text(encoding="utf-8")

    assert '"tag_map_file", "/etc/robotcore/apriltag_map.json"' in source
    assert "tags_ = parse_apriltag_map(root, map_frame_, pool_geometry_)" in source
    assert "if (tags_.empty())" in source
    assert '"empty tag map loaded; absolute Tag localization disabled"' in source
    assert 'root.at("schema_version").get<int>() != 1' in parser
    assert 'root.at("frame").get<std::string>() != expected_frame' in parser
    assert "validate_tag_on_pool" in parser


def test_detector_capacity_is_bounded_and_localizer_has_no_estimator_feedback():
    launch = LAUNCH.read_text(encoding="utf-8")
    cpp = (ROOT / "ros_ws/src/robotcore_sensors/src/apriltag_map_localizer_component.cpp").read_text()

    assert 'DeclareLaunchArgument("apriltag_max_tags", default_value="24")' in launch
    assert 'plugin="robotcore_sensors::AprilTagMapLocalizerComponent"' in launch
    assert "aligned_vio" not in cpp
    assert "nav_msgs/msg/odometry.hpp" not in cpp


def test_zed_nitros_is_enabled_while_operator_video_remains_on_demand():
    launch = LAUNCH.read_text(encoding="utf-8")
    camera_config = (
        ROOT / "ros_ws" / "src" / "robotcore_sensors" / "config" / "zedx_minimal_open.yaml"
    ).read_text(encoding="utf-8")
    assert 'plugin="nvidia::isaac_ros::image_proc::ImageFormatConverterNode"' in launch
    assert 'name="apriltag_cuda_rgb_converter"' in launch
    assert '("image_raw", LaunchConfiguration("front_camera_raw_topic"))' in launch
    assert 'default_value="/localization/apriltag/cuda_input_rgb"' in launch
    assert launch.count(
        '("image", LaunchConfiguration("apriltag_cuda_input_topic"))'
    ) == 2
    assert '"encoding_desired": "rgb8"' in launch
    assert '"image_raw_nitros_format": "nitros_image_bgr8"' in launch
    assert '"image_nitros_format": "nitros_image_rgb8"' in launch
    assert "front_camera_nitros_topic" not in launch
    assert "disable_nitros: false" in camera_config
    assert "jpeg_quality: 80" in camera_config
    assert '"publish_imu_tf:=false enable_ipc:=false node_log_type:=screen "' in launch
    assert "TimerAction" not in launch
    assert "zed_nitros_warmup_s" not in launch


def test_apriltag_payload_remappings_define_one_zed_cuda_localizer_path():
    launch = LAUNCH.read_text(encoding="utf-8")

    # Launch-file position does not impose runtime order. These assertions only
    # lock the shared topic contracts; the deployed endpoint checker validates
    # the actual publisher/subscriber graph.
    assert launch.count('LaunchConfiguration("front_camera_raw_topic")') == 1
    assert launch.count('LaunchConfiguration("front_camera_info_topic")') == 2
    assert launch.count('LaunchConfiguration("apriltag_cuda_input_topic")') == 2
    assert launch.count('LaunchConfiguration("apriltag_detections_topic")') == 2


def test_tag_bootstrap_uses_four_frames_then_enters_the_common_measurement_queue():
    fusion = (
        ROOT
        / "ros_ws/src/robotcore_sensors/src/fixed_lag_eskf_component.cpp"
    ).read_text(encoding="utf-8")

    assert "while (alignment_candidates_.size() > 4U)" in fusion
    assert "if (alignment_candidates_.size() < 4U)" in fusion
    assert "map_from_odom_ = representative_alignment();" in fusion
    assert "event.source = MeasurementSource::Tag" in fusion
    assert "measurements_.insert(insertion, event)" in fusion
    assert "replay_from(*index, event.id)" in fusion
    assert "blend_transform" not in fusion


def test_imu_latched_status_does_not_disable_high_rate_intra_process_path():
    conditioner = (
        ROOT / "ros_ws/src/robotcore_sensors/src/imu_conditioner_component.cpp"
    ).read_text(encoding="utf-8")

    assert "status_options.use_intra_process_comm" in conditioner
    assert "rclcpp::IntraProcessSetting::Disable" in conditioner
