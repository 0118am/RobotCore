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
    assert "void load_map()" in source
    assert "void reload(" in source
    assert "const auto previous = tags_" not in source
    assert "map_error_" not in source
    assert "if (!stream)" not in source


def test_weak_tags_are_removed_without_rejecting_strong_tags_or_logging_acceptance():
    source = LOCALIZER.read_text(encoding="utf-8")
    parser = MAP_PARSER.read_text(encoding="utf-8")

    assert "assess_tag_image_quality(" in source
    assert "shortest < minimum_edge_px" in parser
    assert "longest / shortest > maximum_edge_ratio" in parser
    assert "left.quality.score > right.quality.score" in source
    assert "candidates.resize" in source
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


def test_invalid_map_is_rejected_without_empty_map_runtime_state():
    source = LOCALIZER.read_text(encoding="utf-8")
    parser = MAP_PARSER.read_text(encoding="utf-8")

    assert '"tag_map_file", "/etc/robotcore/apriltag_map.json"' in source
    assert "tags_ = parse_apriltag_map(root, map_frame_, pool_geometry_)" in source
    assert "empty_map_state_published_" not in source
    assert "if (tags_.empty())" not in source
    assert "empty tag map" not in source
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
    assert 'plugin="nvidia::isaac_ros::image_proc::ImageFormatConverterNode"' not in launch
    assert 'name="apriltag_cuda_rgb_converter"' not in launch
    assert "apriltag_cuda_input_topic" not in launch
    assert "/localization/apriltag/cuda_input_rgb" not in launch
    assert '("image", LaunchConfiguration("front_camera_raw_topic"))' in launch
    assert '"encoding_desired": "rgb8"' not in launch
    assert "front_camera_nitros_topic" not in launch
    assert "enable_24bit_output: true" in camera_config
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
    assert "apriltag_cuda_input_topic" not in launch
    assert launch.count('LaunchConfiguration("apriltag_detections_topic")') == 2


def test_tag_measurement_is_replayed_into_vio_tag_ekf():
    fusion = (
        ROOT
        / "ros_ws/src/robotcore_sensors/src/vio_tag_fusion_component.cpp"
    ).read_text(encoding="utf-8")

    assert "candidate =" in fusion
    assert "map_from_base * odom_from_base.inverse()" in fusion
    assert "add_alignment_candidate(candidate" in fusion
    assert "filter.update_position(" in fusion
    assert "filter.update_orientation(" in fusion
    assert "MeasurementSource::Imu" not in fusion
    assert "sensor_msgs::msg::Imu" not in fusion
    assert "MeasurementResult replay(" in fusion


def test_aboard_bridge_publishes_the_canonical_imu_with_bounded_sensor_qos():
    bridge = (
        ROOT / "ros_ws/src/robotcore_hardware/src/aboard_bridge_node.cpp"
    ).read_text(encoding="utf-8")

    assert '"/sensors/external_imu", rclcpp::SensorDataQoS().keep_last(8)' in bridge
    assert 'message.header.frame_id = "base_link"' in bridge
    assert "/hardware/aboard_imu_raw" not in bridge
