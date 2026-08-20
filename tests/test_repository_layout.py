"""Repository boundary tests for the split RobotCore and Web workspaces."""

from pathlib import Path


CORE_ROOT = Path(__file__).resolve().parents[1]
WEB_ROOT = CORE_ROOT.parent / "ControlInterface"


def test_robot_core_owns_all_robot_and_host_components():
    for relative in [
        "ros_ws/src/robotcore_interfaces/package.xml",
        "ros_ws/src/robotcore_bringup/package.xml",
        "ros_ws/src/robotcore_runtime/package.xml",
        "ros_ws/src/robotcore_policy/package.xml",
        "ros_ws/src/robotcore_control/package.xml",
        "ros_ws/src/robotcore_sensors/package.xml",
        "ros_ws/src/robotcore_hardware/package.xml",
        "host_manager/robotcore_host_manager/daemon.py",
        "host_manager/systemd/robotcore.service",
        "host_manager/systemd/robotcore-camera-ipc-ready.service",
        "host_manager/systemd/control-interface.service",
        "scripts/robotcore_topic_check.sh",
    ]:
        assert (CORE_ROOT / relative).exists(), relative

    assert not (CORE_ROOT / "ros_ws/robotcore_mujoco_env").exists()
    assert not any(path.is_file() for path in (CORE_ROOT / "firmware").rglob("*"))


def test_apriltag_map_has_one_json_authority():
    config_dir = CORE_ROOT / "ros_ws/src/robotcore_sensors/config"
    localization = (
        CORE_ROOT / "ros_ws/src/robotcore_sensors/src/apriltag_map_localizer_component.cpp"
    ).read_text(encoding="utf-8")
    parser = (
        CORE_ROOT / "ros_ws/src/robotcore_sensors/include/robotcore_sensors/apriltag_map.hpp"
    ).read_text(encoding="utf-8")
    edge_launch = (
        CORE_ROOT / "ros_ws/src/robotcore_bringup/launch/robotcore_edge_system.launch.py"
    ).read_text(encoding="utf-8")

    assert not (config_dir / "tank_apriltag_map.json").exists()
    assert not (config_dir / "tank_apriltag_map.yaml").exists()
    assert 'default_value="/etc/robotcore/apriltag_map.json"' in edge_launch
    assert '"tag_map_file", "/etc/robotcore/apriltag_map.json"' in localization
    assert "parse_apriltag_map(root, map_frame_, pool_geometry_)" in localization
    assert "nlohmann::json" in parser
    assert "YAML" not in parser


def test_apriltag_localizer_publishes_one_quality_gated_absolute_measurement():
    localization = (CORE_ROOT / "ros_ws/src/robotcore_sensors/src/apriltag_map_localizer_component.cpp").read_text()
    fusion = (CORE_ROOT / "ros_ws/src/robotcore_sensors/src/vio_tag_fusion_component.cpp").read_text()
    edge_launch = (
        CORE_ROOT / "ros_ws/src/robotcore_bringup/launch/robotcore_edge_system.launch.py"
    ).read_text(encoding="utf-8")

    assert '"minimum_pose_tag_count", 1' in localization
    assert '"minimum_inlier_corners_per_tag", 3' in localization
    assert '"minimum_pose_tag_count": 1' in edge_launch
    assert '"single_tag_max_reprojection_rms_px", 1.5' in localization
    assert '"single_tag_max_reprojection_rms_px": 1.5' in edge_launch
    assert "inliers = (cv::Mat_<int>(4, 1) << 0, 1, 2, 3)" in localization
    assert "independently_supported_tag_indices(" in localization
    assert "estimate.inlier_tag_count < min_tags_" in localization
    assert "cv::solvePnPRefineLM" in localization
    assert "single_tag_reference_edge_px_ / minimum_edge" in localization
    assert "estimate_pub_->publish(estimate)" in localization
    assert "solvePnPRansac" in localization
    assert "cv::SOLVEPNP_IPPE" in localization
    assert "empty_map_state_published_" not in localization
    assert "if (tags_.empty())" not in localization
    assert '"/localization/apriltag_pose_degraded"' not in localization
    assert "aligned_vio" not in localization
    assert "sample_at(tag_stamp_ns)" in fusion
    assert "update_alignment(candidate" in fusion


def test_apriltag_reprojection_and_alignment_innovation_gates_are_explicit_in_cpp():
    localization = (CORE_ROOT / "ros_ws/src/robotcore_sensors/src/apriltag_map_localizer_component.cpp").read_text()
    fusion = (CORE_ROOT / "ros_ws/src/robotcore_sensors/src/vio_tag_fusion_component.cpp").read_text()
    edge_launch = (
        CORE_ROOT / "ros_ws/src/robotcore_bringup/launch/robotcore_edge_system.launch.py"
    ).read_text(encoding="utf-8")

    assert '"max_reprojection_rms_px", 3.0' in localization
    assert '"max_reprojection_rms_px": 3.0' in edge_launch
    assert "solvePnPRansac" in localization
    assert '"multi_tag_position_stddev_m", 0.05' in localization
    assert '"multi_tag_position_stddev_m": 0.05' in edge_launch
    assert "max_translation_jump_m" not in localization
    assert "max_translation_jump_m" not in edge_launch
    assert '"tag_innovation_gate_m", 0.50' in fusion
    assert "last_tag_translation_residual_ > tag_innovation_gate_m_" in fusion
    assert "++tag_gate_rejections_" in fusion
    assert '"use_vio"' not in edge_launch
    assert '"vio_arrival_timeout_s": 0.30' in edge_launch
    assert '"vio_prediction_horizon_s": 0.50' in edge_launch
    assert 'plugin="robotcore_sensors::VioTagFusionComponent"' in edge_launch
    assert "continuous_rejection_reanchor_required(" not in fusion
    assert "replay_from(" not in fusion
    assert "vio_reanchors" not in fusion


def test_apriltag_relocalize_is_folded_into_the_single_estimate_stream():
    localization = (
        CORE_ROOT / "ros_ws/src/robotcore_sensors/src/apriltag_map_localizer_component.cpp"
    ).read_text(encoding="utf-8")
    fusion = (
        CORE_ROOT / "ros_ws/src/robotcore_sensors/src/vio_tag_fusion_component.cpp"
    ).read_text(encoding="utf-8")

    assert localization.count("++map_generation_") == 1
    assert "estimate.relocalization_requested = true" in localization
    assert '"/localization/relocalize_event"' not in localization
    assert "estimator_relocalize_client_" not in localization
    assert "relocalize_event_sub_" not in localization
    assert '"/localization/tag_vio/relocalize"' not in localization
    assert '"/localization/relocalize_event"' not in fusion
    assert "relocalize_event_sub_" not in fusion
    assert "relocalize_event_pub_" not in fusion
    assert "message->map_generation > last_tag_map_generation_" in fusion
    assert "message->relocalization_requested || map_changed" in fusion
    assert fusion.count("tag_sub_ = create_subscription") == 1


def test_camera_extrinsic_has_one_tf_authority():
    localization = (
        CORE_ROOT / "ros_ws/src/robotcore_sensors/src/apriltag_map_localizer_component.cpp"
    ).read_text(encoding="utf-8")
    edge_launch = (
        CORE_ROOT / "ros_ws/src/robotcore_bringup/launch/robotcore_edge_system.launch.py"
    ).read_text(encoding="utf-8")

    assert "lookupTransform(base_frame_, camera_frame_, tf2::TimePointZero)" in localization
    assert "camera_ready_ && camera_extrinsic_ready_" in localization
    assert "base_to_camera_translation_m" not in localization
    assert "base_to_camera_optical_rpy_rad" not in localization
    assert "base_to_front_camera_optical" not in edge_launch
    assert "front_camera_optical_frame" not in edge_launch


def test_apriltag_image_path_is_bounded_and_localizer_consumes_only_detections():
    localization = (
        CORE_ROOT / "ros_ws/src/robotcore_sensors/src/apriltag_map_localizer_component.cpp"
    ).read_text(encoding="utf-8")
    edge_launch = (
        CORE_ROOT / "ros_ws/src/robotcore_bringup/launch/robotcore_edge_system.launch.py"
    ).read_text(encoding="utf-8")
    camera_config = (
        CORE_ROOT / "ros_ws/src/robotcore_sensors/config/zedx_minimal_open.yaml"
    ).read_text(encoding="utf-8")

    assert "AprilTagDetectionArray" in localization
    assert "sensor_msgs/msg/image" not in localization
    assert "SensorDataQoS().keep_last(1)" in localization
    assert "pub_resolution: CUSTOM" in camera_config
    assert "grab_resolution: SVGA" in camera_config
    assert "pub_downscale_factor: 1.0" in camera_config
    assert "publish_imu: false" in camera_config
    assert "enable_24bit_output: true" in camera_config
    assert 'default_value="/zedx/zed_node/rgb/color/rect/image"' in edge_launch
    assert 'default_value="/zedx/zed_node/rgb/color/rect/image/compressed"' in edge_launch
    assert '("image_raw", LaunchConfiguration("front_camera_raw_topic"))' in edge_launch
    assert edge_launch.count(
        '("image", LaunchConfiguration("apriltag_cuda_input_topic"))'
    ) == 2
    assert '"front_camera_compressed_topic": LaunchConfiguration(' in edge_launch
    assert '".zed_node":' in camera_config
    assert "jpeg_quality: 80" in camera_config


def test_web_workspace_keeps_only_web_package_and_ros_state_bridge():
    for relative in [
        "README.md",
        "control_interface/control_interface/web_operator_node.py",
        "control_interface/control_interface/web_server.py",
        "control_interface/control_interface/web_state.py",
        "control_interface/static/index.html",
        "control_interface/static/app.js",
        "control_interface/static/styles.css",
    ]:
        assert (WEB_ROOT / relative).exists(), relative
    assert not (WEB_ROOT / "src").exists()
    assert not (WEB_ROOT / "robotcore_host_manager").exists()


def test_web_bridge_consumes_robot_core_contract_without_owning_devices_or_maps():
    bridge = (WEB_ROOT / "control_interface/control_interface/web_operator_node.py").read_text(encoding="utf-8")
    host_manager = (CORE_ROOT / "host_manager/robotcore_host_manager/daemon.py").read_text(encoding="utf-8")
    robot_unit = (CORE_ROOT / "host_manager/systemd/robotcore.service").read_text(encoding="utf-8")
    web_unit = (CORE_ROOT / "host_manager/systemd/control-interface.service").read_text(encoding="utf-8")
    edge_launch = (CORE_ROOT / "ros_ws/src/robotcore_bringup/launch/robotcore_edge_system.launch.py").read_text(
        encoding="utf-8"
    )

    assert "from robotcore_interfaces.msg import" in bridge
    assert "host_manager_socket" in bridge
    assert "apriltag-upsert" in host_manager
    assert "apriltag-delete" in host_manager
    assert "ROBOTCORE_WORKSPACE" in robot_unit
    assert "SupplementaryGroups=robotops video render dialout" in robot_unit
    assert "CONTROL_INTERFACE_WORKSPACE" in web_unit
    assert "imu_topic:=" not in web_unit
    assert "dialout" not in web_unit
    assert "exec ros2 launch zed_wrapper zed_camera.launch.py" in edge_launch
    assert 'LaunchConfiguration("zed_serial_number")' in edge_launch
    for forbidden in (
        "manual_thruster_serial",
        "manual_thruster_span_us",
        "manual_thruster_channel_offset",
        "build_direct_pwm_frame",
        "_write_manual_serial",
    ):
        assert forbidden not in bridge

    hardware_node_start = edge_launch.index('package="robotcore_hardware"')
    web_node_start = edge_launch.index('package="control_interface"')
    hardware_node = edge_launch[hardware_node_start:web_node_start]
    web_node = edge_launch[web_node_start:]
    assert '"span_us": ParameterValue(' in hardware_node
    assert 'LaunchConfiguration("manual_thruster_span_us")' in hardware_node
    assert "manual_thruster_span_us" not in web_node


def test_robotcore_aboard_rule_matches_the_detected_cdc_acm_board():
    rule = (CORE_ROOT / "host_manager/systemd/99-robotcore-aboard.rules").read_text(encoding="utf-8")

    assert 'KERNEL=="ttyACM*"' in rule
    assert 'ATTRS{idVendor}=="1a86"' in rule
    assert 'ATTRS{idProduct}=="55d3"' in rule
    assert 'SYMLINK+="robotcore/aboard"' in rule
    assert 'GROUP="dialout"' in rule


def test_only_production_aboard_bridge_uses_protocol_v2():
    hardware_package = CORE_ROOT / "ros_ws/src/robotcore_hardware"
    bridge = (hardware_package / "src/aboard_bridge_node.cpp").read_text(encoding="utf-8")
    board_status = (CORE_ROOT / "ros_ws/src/robotcore_interfaces/msg/BoardStatus.msg").read_text(
        encoding="utf-8"
    )

    assert not (hardware_package / "robotcore_hardware/aboard_bridge_node.py").exists()
    assert not (CORE_ROOT / "ros_ws/src/robotcore_interfaces/msg/ThrusterState.msg").exists()
    assert "build_command_v2" in bridge
    assert "parse_board_status_v2" in bridge
    assert "uint8 protocol_version" in board_status
    assert "bool imu_gyro_calibration_active" in board_status
    assert "bool imu_gyro_calibration_succeeded" in board_status
    assert "bool imu_gyro_calibration_failed" in board_status
    assert "status.imu_gyro_calibration_active" in bridge
    assert "status.imu_gyro_calibration_succeeded" in bridge
    assert "status.imu_gyro_calibration_failed" in bridge
    assert '"/hardware/aboard/calibrate_gyro"' in bridge
    assert "forced_stop" not in board_status
    assert "estop_active" not in board_status
    assert "firmware_version" not in board_status
    assert "/robot/thruster_state" not in bridge
    assert "get_publishers_info_by_topic" in bridge
    assert "publisher_gid" not in bridge
    assert "MessageInfo" not in bridge
    assert "command_authority freshness timeout" in bridge
    assert "kCommandPeriod = 20ms" in bridge
    assert "declare_parameter<std::int64_t>(\"span_us\", 500)" in bridge
    assert "requested_span_us, 1, 500" in bridge
    assert "kMinimumReportedPwmUs = 1000U" in bridge
    assert "kMaximumReportedPwmUs = 2000U" in bridge
    assert "message->normalized[i]) * span_us_" in bridge
    assert "diagnostic_timer_ = create_wall_timer(1s" in bridge
    assert bridge.count("updater_.force_update()") == 1


def test_pool_bottom_frame_has_no_unvalidated_depth_sensor_path():
    body_state = (CORE_ROOT / "ros_ws/src/robotcore_interfaces/msg/BodyState.msg").read_text(
        encoding="utf-8"
    )
    fusion = (CORE_ROOT / "ros_ws/src/robotcore_sensors/src/vio_tag_fusion_component.cpp").read_text(
        encoding="utf-8"
    )

    assert "depth_m" not in body_state
    assert "altitude_m" not in body_state
    assert "depth_input_topic" not in fusion
    assert "altitude_input_topic" not in fusion


def test_operator_telemetry_uses_canonical_body_and_imu_topics():
    conditioner = (
        CORE_ROOT / "ros_ws/src/robotcore_sensors/src/imu_conditioner_component.cpp"
    ).read_text(encoding="utf-8")
    fusion = (
        CORE_ROOT / "ros_ws/src/robotcore_sensors/src/vio_tag_fusion_component.cpp"
    ).read_text(encoding="utf-8")
    authority = (
        CORE_ROOT / "ros_ws/src/robotcore_control_cpp/src/command_authority_node.cpp"
    ).read_text(encoding="utf-8")
    web = (
        WEB_ROOT / "control_interface/control_interface/web_operator_node.py"
    ).read_text(encoding="utf-8")

    assert '"/ui/external_imu"' not in conditioner
    assert '"/ui/body_state"' not in fusion
    assert '"ui_command_topic", "/ui/thruster_cmd"' in authority
    assert 'self.declare_parameter("body_topic", "/robot/body_state")' in web
    assert 'self.declare_parameter("imu_topic", "/sensors/external_imu")' in web
    assert 'self.declare_parameter("thruster_command_topic", "/ui/thruster_cmd")' in web


def test_cpp_localization_has_no_uninstalled_python_shadow_implementation():
    legacy_package = CORE_ROOT / "ros_ws/src/robotcore_sensors/robotcore_sensors"
    legacy_config = CORE_ROOT / "ros_ws/src/robotcore_sensors/config/sensors.yaml"

    assert not any(legacy_package.glob("*.py"))
    assert not legacy_config.exists()
