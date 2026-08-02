"""Static acceptance tests for the edge localisation pipeline."""

from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
SENSORS = ROOT / "ros_ws/src/eup_sensors"


def load_yaml(relative: str):
    return yaml.safe_load((SENSORS / relative).read_text(encoding="utf-8"))


def test_zed_camera_uses_one_fixed_30_hz_frame_rate():
    parameters = load_yaml("config/zedx_minimal_open.yaml")["/**"]["ros__parameters"]

    assert parameters["general"]["grab_frame_rate"] == 30
    assert parameters["general"]["pub_frame_rate"] == 30.0
    assert parameters["sensors"]["sensors_pub_rate"] == 10.0
    assert parameters["pos_tracking"]["imu_fusion"] is True
    assert parameters["pos_tracking"]["publish_odom_pose"] is True


def test_fixed_rate_ekf_fuses_visual_state_and_only_external_gyro():
    parameters = load_yaml("config/tag_vio_external_imu_ekf.yaml")[
        "localization_ekf"
    ]["ros__parameters"]

    assert parameters["frequency"] == 30.0
    assert parameters["world_frame"] == "map"
    assert parameters["odom0"] == "/localization/aligned_vio_odom"
    assert parameters["imu0"] == "/sensors/external_imu"
    assert parameters["publish_tf"] is False
    assert parameters["publish_acceleration"] is False

    # State order: x/y/z, roll/pitch/yaw, vx/vy/vz, vroll/vpitch/vyaw, ax/ay/az.
    assert parameters["odom0_config"] == [True] * 9 + [False] * 6
    assert parameters["imu0_config"] == [False] * 9 + [True] * 3 + [False] * 3
    assert parameters["imu0_remove_gravitational_acceleration"] is False
    assert "odom1" not in parameters
    assert "pose0" not in parameters
    assert "twist0" not in parameters


def test_external_imu_requires_stationary_bias_calibration():
    config = load_yaml("config/external_imu.yaml")
    parameters = config["imu_conditioning"]["ros__parameters"]
    frames = config["vehicle_frames"]["ros__parameters"]

    assert parameters["input_topic"] == "/hardware/aboard_imu_raw"
    assert parameters["output_topic"] == "/sensors/external_imu"
    assert parameters["status_topic"] == "/localization/external_imu_ready"
    assert parameters["auto_start"] is True
    assert parameters["calibration_sample_count"] >= 200
    assert frames["base_to_imu_translation_m"] == [0.018, 0.0, 0.076]
    assert len(frames["base_to_imu_rpy_rad"]) == 3


def test_edge_launch_wires_one_canonical_fixed_rate_output():
    launch = (
        ROOT / "ros_ws/src/eup_bringup/launch/eup_edge_system.launch.py"
    ).read_text(encoding="utf-8")

    assert 'executable="imu_conditioning_node"' in launch
    assert '"imu_topic": LaunchConfiguration("zed_imu_topic")' in launch
    assert '"publish_imu": True' in launch
    assert '"output_odometry_topic": "/localization/aligned_vio_odom"' in launch
    assert "pressure_depth_odometry_node" not in launch
    assert "/localization/pressure_depth_odom" not in launch
    assert 'package="robot_localization"' in launch
    assert 'name="localization_ekf"' in launch
    assert '("odometry/filtered", "/localization/fused_odom")' in launch
    assert '"fused_odometry_topic": "/localization/fused_odom"' in launch


def test_body_state_marks_velocity_stale_when_zed_vio_stops():
    fusion = (
        SENSORS / "eup_sensors/sensor_fusion_node.py"
    ).read_text(encoding="utf-8")

    assert "body.linear_velocity_valid and vio_fresh" in fusion
    assert "body.position_estimated = vio_fresh and not tag_accepted" in fusion
    assert "reliability=ReliabilityPolicy.BEST_EFFORT" in fusion
    assert "source_stamp_ns <= self.zed_odometry_stamp_ns" in fusion


def test_localization_status_exposes_rate_age_innovation_and_covariance():
    interface = (
        ROOT / "ros_ws/src/eup_interfaces/msg/LocalizationStatus.msg"
    ).read_text(encoding="utf-8")
    cmake = (
        ROOT / "ros_ws/src/eup_interfaces/CMakeLists.txt"
    ).read_text(encoding="utf-8")
    logger = (
        ROOT / "ros_ws/src/eup_runtime/eup_runtime/run_logger.py"
    ).read_text(encoding="utf-8")

    assert '"msg/LocalizationStatus.msg"' in cmake
    for field in (
        "vio_age_s",
        "tag_age_s",
        "vio_rate_hz",
        "tag_rate_hz",
        "fused_rate_hz",
        "apriltag_frame_rate_hz",
        "apriltag_frame_age_s",
        "tag_vio_translation_residual_m",
        "tag_vio_angle_residual_deg",
        "mapped_tag_count",
        "inlier_tag_count",
        "tag_reprojection_rms_px",
        "apriltag_rejection_reason",
        "pose_covariance",
        "twist_covariance",
        "rejection_reason",
    ):
        assert field in interface
    assert '"/localization/status"' in logger
    assert '"localization_status"' in logger
    assert '"msg/AprilTagPoseStatus.msg"' in cmake
