"""Repository boundary tests for the split RobotCore and Web workspaces."""

from pathlib import Path


CORE_ROOT = Path(__file__).resolve().parents[1]
WEB_ROOT = CORE_ROOT.parent / "ControlInterface"


def test_robot_core_owns_all_robot_and_host_components():
    for relative in [
        "ros_ws/src/eup_interfaces/package.xml",
        "ros_ws/src/eup_bringup/package.xml",
        "ros_ws/src/eup_runtime/package.xml",
        "ros_ws/src/eup_policy/package.xml",
        "ros_ws/src/eup_control/package.xml",
        "ros_ws/src/eup_sensors/package.xml",
        "ros_ws/src/eup_hardware/package.xml",
        "ros_ws/eup_mujoco_env/package.xml",
        "host_manager/robotcore_host_manager/daemon.py",
        "host_manager/systemd/robotcore.service",
        "host_manager/systemd/control-interface.service",
        "firmware/src/main.c",
        "scripts/robotcore_topic_check.sh",
    ]:
        assert (CORE_ROOT / relative).exists(), relative


def test_apriltag_map_has_one_json_authority():
    config_dir = CORE_ROOT / "ros_ws/src/eup_sensors/config"
    localization = (
        CORE_ROOT / "ros_ws/src/eup_sensors/eup_sensors/apriltag_localization_node.py"
    ).read_text(encoding="utf-8")
    edge_launch = (
        CORE_ROOT / "ros_ws/src/eup_bringup/launch/eup_edge_system.launch.py"
    ).read_text(encoding="utf-8")

    assert not (config_dir / "tank_apriltag_map.json").exists()
    assert not (config_dir / "tank_apriltag_map.yaml").exists()
    assert 'default_value="/etc/robotcore/apriltag_map.json"' in edge_launch
    assert "json.loads" in localization
    assert "yaml.safe_load" not in localization


def test_apriltag_pose_separates_trusted_alignment_from_two_tag_validation():
    localization = (
        CORE_ROOT / "ros_ws/src/eup_sensors/eup_sensors/apriltag_localization_node.py"
    ).read_text(encoding="utf-8")
    alignment = (
        CORE_ROOT / "ros_ws/src/eup_sensors/eup_sensors/tag_vio_alignment_node.py"
    ).read_text(encoding="utf-8")
    fusion = (
        CORE_ROOT / "ros_ws/src/eup_sensors/eup_sensors/sensor_fusion_node.py"
    ).read_text(encoding="utf-8")
    edge_launch = (
        CORE_ROOT / "ros_ws/src/eup_bringup/launch/eup_edge_system.launch.py"
    ).read_text(encoding="utf-8")

    assert 'self.declare_parameter("minimum_pose_tag_count", 3)' in localization
    assert '"minimum_inlier_corners_per_tag", 3' in localization
    assert '"degraded_two_tag_inlier_corners_per_tag", 4' in localization
    assert '"degraded_two_tag_max_full_rms_px", 3.0' in localization
    assert '"/localization/apriltag_pose_degraded"' in localization
    assert '"/localization/apriltag_pose_degraded"' in fusion
    assert '"/localization/apriltag_pose_degraded"' not in alignment
    assert "degraded=True," in localization
    assert "self.pose_uncertainty_scale(" in localization
    assert "preferred_single_tag_correspondence" not in localization
    assert '"minimum_pose_tag_count": 3' in edge_launch
    assert '"degraded_two_tag_inlier_corners_per_tag": 4' in edge_launch
    degraded_block = localization[
        localization.index("def publish_degraded_two_tag_pose(")
        : localization.index("def tag_map_uncertainty_allowance_px(")
    ]
    assert "reset_pose_filter" not in degraded_block
    assert "confirm_reacquisition" not in degraded_block
    assert "filter_map_from_base" not in degraded_block


def test_apriltag_reprojection_gate_allows_five_centimetres_of_map_uncertainty():
    localization = (
        CORE_ROOT / "ros_ws/src/eup_sensors/eup_sensors/apriltag_localization_node.py"
    ).read_text(encoding="utf-8")
    edge_launch = (
        CORE_ROOT / "ros_ws/src/eup_bringup/launch/eup_edge_system.launch.py"
    ).read_text(encoding="utf-8")

    assert 'self.declare_parameter("max_reprojection_rms_px", 3.0)' in localization
    assert '"max_reprojection_rms_px": 3.0' in edge_launch
    assert 'self.declare_parameter("tag_map_position_uncertainty_m", 0.05)' in localization
    assert "self.tag_map_uncertainty_allowance_px(image_points, seen_ids)" in localization
    assert '"tag_map_position_uncertainty_m": 0.05' in edge_launch
    assert '"enforce_cuboid_pool_geometry": True' in edge_launch
    assert '"pool_length_m": 5.42' in edge_launch
    assert '"pool_width_m": 3.73' in edge_launch
    assert 'self.declare_parameter("max_translation_jump_m", 0.05)' in localization
    assert '"max_translation_jump_m": 0.05' in edge_launch
    assert 'self.declare_parameter("multi_tag_position_stddev_m", 0.05)' in localization
    assert '"multi_tag_position_stddev_m": 0.05' in edge_launch


def test_apriltag_relocalize_bypasses_the_old_pose_jump_gate_once():
    localization = (
        CORE_ROOT / "ros_ws/src/eup_sensors/eup_sensors/apriltag_localization_node.py"
    ).read_text(encoding="utf-8")

    assert "self.relocalization_pending = True" in localization
    transition_block = localization[
        localization.index("map_from_base = map_from_camera")
        : localization.index("map_from_base = self.filter_map_from_base")
    ]
    assert "if self.relocalization_pending:" in transition_block
    assert transition_block.index("if self.relocalization_pending:") < transition_block.index(
        'self.runtime_parameters["enforce_transition_gate"]'
    )
    assert "self.relocalization_pending = False" in transition_block


def test_apriltag_image_path_is_bounded_and_debug_encoding_is_not_inline():
    localization = (
        CORE_ROOT / "ros_ws/src/eup_sensors/eup_sensors/apriltag_localization_node.py"
    ).read_text(encoding="utf-8")
    edge_launch = (
        CORE_ROOT / "ros_ws/src/eup_bringup/launch/eup_edge_system.launch.py"
    ).read_text(encoding="utf-8")
    camera_config = (
        CORE_ROOT / "ros_ws/src/eup_sensors/config/zedx_minimal_open.yaml"
    ).read_text(encoding="utf-8")

    assert "queue_matching_debug_image" not in localization
    assert "debug_image_pub" not in localization
    assert 'self.warn_throttled(' in localization
    assert '"pnp-rejected"' in localization
    assert "largest_tag_quads" in localization
    assert "isaac_ros_tag36h11_corners_in_map_axis_order(detected_corners)" in localization
    assert "pub_resolution: CUSTOM" in camera_config
    assert "pub_downscale_factor: 2.0" in camera_config
    assert "enable_24bit_output: true" in camera_config
    assert 'default_value="/zedx/zed_node/rgb/color/rect/image"' in edge_launch
    assert 'default_value="/zedx/zed_node/rgb/color/rect/image/compressed"' in edge_launch
    assert '("image", LaunchConfiguration("front_camera_raw_topic"))' in edge_launch
    assert '"front_camera_compressed_topic": LaunchConfiguration(' in edge_launch
    assert '".zed_node":' in camera_config
    assert "jpeg_quality: 30" in camera_config


def test_web_workspace_keeps_only_web_package_and_ros_state_bridge():
    for relative in [
        "README.md",
        "eup_ui/eup_ui/web_operator_node.py",
        "eup_ui/eup_ui/web_server.py",
        "eup_ui/eup_ui/web_state.py",
        "eup_ui/static/index.html",
        "eup_ui/static/app.js",
        "eup_ui/static/styles.css",
    ]:
        assert (WEB_ROOT / relative).exists(), relative
    assert not (WEB_ROOT / "src").exists()
    assert not (WEB_ROOT / "robotcore_host_manager").exists()


def test_web_bridge_consumes_robot_core_contract_without_owning_devices_or_maps():
    bridge = (WEB_ROOT / "eup_ui/eup_ui/web_operator_node.py").read_text(encoding="utf-8")
    host_manager = (CORE_ROOT / "host_manager/robotcore_host_manager/daemon.py").read_text(encoding="utf-8")
    robot_unit = (CORE_ROOT / "host_manager/systemd/robotcore.service").read_text(encoding="utf-8")
    web_unit = (CORE_ROOT / "host_manager/systemd/control-interface.service").read_text(encoding="utf-8")
    edge_launch = (CORE_ROOT / "ros_ws/src/eup_bringup/launch/eup_edge_system.launch.py").read_text(
        encoding="utf-8"
    )

    assert "from eup_interfaces.msg import" in bridge
    assert "host_manager_socket" in bridge
    assert "apriltag-upsert" in host_manager
    assert "apriltag-delete" in host_manager
    assert "ROBOTCORE_WORKSPACE" in robot_unit
    assert "SupplementaryGroups=robotops video render dialout" in robot_unit
    assert "CONTROL_INTERFACE_WORKSPACE" in web_unit
    assert "zed_camera_launcher" in edge_launch
    assert 'LaunchConfiguration("zed_serial_number"), value_type=str' in edge_launch


def test_robotcore_aboard_rule_matches_the_detected_cdc_acm_board():
    rule = (CORE_ROOT / "host_manager/systemd/99-robotcore-aboard.rules").read_text(encoding="utf-8")

    assert 'KERNEL=="ttyACM*"' in rule
    assert 'ATTRS{idVendor}=="1a86"' in rule
    assert 'ATTRS{idProduct}=="55d3"' in rule
    assert 'SYMLINK+="robotcore/aboard"' in rule
    assert 'GROUP="dialout"' in rule


def test_board_status_does_not_claim_a_hard_coded_firmware_revision():
    bridge = (CORE_ROOT / "ros_ws/src/eup_hardware/eup_hardware/aboard_bridge_node.py").read_text(encoding="utf-8")

    assert 'status.firmware_version = "aboard-uart6-real"' not in bridge
    assert 'status.firmware_version = ""' in bridge
