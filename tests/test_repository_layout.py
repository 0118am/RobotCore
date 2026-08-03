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


def test_apriltag_pose_separates_trusted_alignment_from_two_tag_validation():
    localization = (CORE_ROOT / "ros_ws/src/robotcore_sensors/src/apriltag_map_localizer_component.cpp").read_text()
    fusion = (CORE_ROOT / "ros_ws/src/robotcore_sensors/src/fixed_lag_eskf_component.cpp").read_text()
    edge_launch = (
        CORE_ROOT / "ros_ws/src/robotcore_bringup/launch/robotcore_edge_system.launch.py"
    ).read_text(encoding="utf-8")

    assert '"minimum_pose_tag_count", 3' in localization
    assert '"minimum_inlier_corners_per_tag", 3' in localization
    assert '"/localization/apriltag_pose_degraded"' in localization
    assert '"/localization/apriltag_pose_degraded"' not in fusion
    assert "const bool degraded = seen_ids.size() == 2U" in localization
    assert "degraded_consistent" in localization
    assert '"minimum_pose_tag_count": 3' in edge_launch
    assert "(degraded ? degraded_pub_ : pose_pub_)->publish(pose)" in localization


def test_apriltag_reprojection_and_transition_gates_are_explicit_in_cpp():
    localization = (CORE_ROOT / "ros_ws/src/robotcore_sensors/src/apriltag_map_localizer_component.cpp").read_text()
    edge_launch = (
        CORE_ROOT / "ros_ws/src/robotcore_bringup/launch/robotcore_edge_system.launch.py"
    ).read_text(encoding="utf-8")

    assert '"max_reprojection_rms_px", 3.0' in localization
    assert '"max_reprojection_rms_px": 3.0' in edge_launch
    assert "solvePnPRansac" in localization
    assert '"max_translation_jump_m", 0.05' in localization
    assert '"max_translation_jump_m": 0.05' in edge_launch
    assert '"multi_tag_position_stddev_m", 0.05' in localization
    assert '"multi_tag_position_stddev_m": 0.05' in edge_launch


def test_apriltag_relocalize_bypasses_the_old_pose_jump_gate_once():
    localization = (
        CORE_ROOT / "ros_ws/src/robotcore_sensors/src/apriltag_map_localizer_component.cpp"
    ).read_text(encoding="utf-8")

    assert "have_last_pose_ = false; relocalization_pending_ = true" in localization
    assert "have_last_pose_ && !relocalization_pending_" in localization
    assert "relocalization_pending_ = false" in localization


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
    assert "pub_downscale_factor: 2.0" in camera_config
    assert "enable_24bit_output: true" in camera_config
    assert 'default_value="/zedx/zed_node/rgb/color/rect/image"' in edge_launch
    assert 'default_value="/zedx/zed_node/rgb/color/rect/image/compressed"' in edge_launch
    assert '("image_raw", LaunchConfiguration("front_camera_raw_topic"))' in edge_launch
    assert '("image", "/localization/apriltag/image_rgb")' in edge_launch
    assert '"front_camera_compressed_topic": LaunchConfiguration(' in edge_launch
    assert '".zed_node":' in camera_config
    assert "jpeg_quality: 30" in camera_config


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
    assert "imu_topic:=/sensors/external_imu" in web_unit
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
    assert "forced_stop" not in board_status
    assert "estop_active" not in board_status
    assert "firmware_version" not in board_status
    assert "/robot/thruster_state" not in bridge
    assert "get_publishers_info_by_topic" in bridge
    assert "publisher_gid" not in bridge
    assert "MessageInfo" not in bridge
    assert "command_authority freshness timeout" in bridge
    assert "kCommandPeriod = 20ms" in bridge


def test_pool_bottom_frame_has_no_unvalidated_depth_sensor_path():
    body_state = (CORE_ROOT / "ros_ws/src/robotcore_interfaces/msg/BodyState.msg").read_text(
        encoding="utf-8"
    )
    fusion = (
        CORE_ROOT / "ros_ws/src/robotcore_sensors/robotcore_sensors/sensor_fusion_node.py"
    ).read_text(encoding="utf-8")
    sensor_config = (CORE_ROOT / "ros_ws/src/robotcore_sensors/config/sensors.yaml").read_text(
        encoding="utf-8"
    )

    assert "depth_m" not in body_state
    assert "altitude_m" not in body_state
    assert "depth_input_topic" not in fusion
    assert "altitude_input_topic" not in fusion
    assert "aboard_depth" not in sensor_config
