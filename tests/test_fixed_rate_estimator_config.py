"""Static acceptance tests for the edge localisation pipeline."""

from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
SENSORS = ROOT / "ros_ws/src/robotcore_sensors"


def load_yaml(relative: str):
    return yaml.safe_load((SENSORS / relative).read_text(encoding="utf-8"))


def test_zed_camera_uses_one_fixed_30_hz_frame_rate():
    parameters = load_yaml("config/zedx_minimal_open.yaml")["/**"]["ros__parameters"]

    assert parameters["general"]["grab_frame_rate"] == 30
    assert parameters["general"]["pub_frame_rate"] == 30.0
    assert parameters["sensors"]["sensors_pub_rate"] == 10.0
    assert parameters["pos_tracking"]["imu_fusion"] is True
    assert parameters["pos_tracking"]["publish_odom_pose"] is True


def test_fixed_lag_eskf_is_native_cpp_and_has_the_planned_state_and_updates():
    header = (SENSORS / "include/robotcore_sensors/fixed_lag_eskf.hpp").read_text()
    source = (SENSORS / "src/fixed_lag_eskf.cpp").read_text()
    cmake = (SENSORS / "CMakeLists.txt").read_text()

    for field in ("position", "velocity", "orientation", "gyro_bias", "accel_bias"):
        assert field in header
    assert "Matrix15d" in header
    assert "update_pose" in source
    assert "update_velocity" in source
    assert "correction * state_.covariance * correction.transpose()" in source
    assert "-O3" in cmake
    assert "CMAKE_INTERPROCEDURAL_OPTIMIZATION" in cmake
    assert "ffast-math" not in cmake
    assert not (SENSORS / "config/tag_vio_external_imu_ekf.yaml").exists()


def test_external_imu_requires_stationary_bias_calibration():
    config = load_yaml("config/external_imu.yaml")
    parameters = config["imu_conditioning"]["ros__parameters"]

    assert parameters["input_topic"] == "/hardware/aboard_imu_raw"
    assert parameters["output_topic"] == "/sensors/external_imu"
    assert parameters["fusion_output_topic"] == "/sensors/external_imu_specific_force"
    assert parameters["status_topic"] == "/localization/external_imu_ready"
    assert parameters["calibration_sample_count"] == 250
    assert parameters["calibration_timeout_s"] == 10.0
    assert parameters["calibration_file"] == "/etc/robotcore/external_imu_calibration.yaml"
    assert "auto_start" not in parameters
    assert len(parameters["base_to_imu_rpy_rad"]) == 3
    assert parameters["base_frame_id"] == "base_link"

    conditioner = (SENSORS / "src/imu_conditioner_component.cpp").read_text()
    assert "waiting for operator calibration" in conditioner
    assert "if (!collecting_) {return;}" in conditioner
    assert "startup_accel_baseline_ - expected" in conditioner
    assert "accel - startup_accel_baseline" in conditioner
    assert "fusion_output_pub_->publish(fusion_output)" in conditioner
    assert "output.header.frame_id = base_frame_id_" in conditioner
    assert "rotate_imu_vector_to_base(corrected_gyro, base_from_imu_)" in conditioner
    assert "persistent_calibration_valid_, startup_accel_residual_.norm()" in conditioner
    assert "have_vio_orientation" not in conditioner
    assert "publish_ready();" in conditioner


def test_edge_launch_wires_one_canonical_fixed_rate_output():
    launch = (
        ROOT / "ros_ws/src/robotcore_bringup/launch/robotcore_edge_system.launch.py"
    ).read_text(encoding="utf-8")

    assert 'plugin="robotcore_sensors::ImuConditionerComponent"' in launch
    assert 'plugin="robotcore_sensors::ZedOdometryAdapterComponent"' in launch
    assert 'plugin="robotcore_sensors::FixedLagEskfComponent"' in launch
    assert '"vio_topic": "/localization/zed_odom"' in launch
    assert '"imu_topic": LaunchConfiguration("external_imu_fusion_topic")' in launch
    assert '"output_rate_hz": 60.0' in launch
    assert "pressure_depth_odometry_node" not in launch
    assert "/localization/pressure_depth_odom" not in launch
    assert 'package="robot_localization"' not in launch
    assert "sensor_fusion_node" not in launch


def test_body_state_marks_velocity_stale_when_zed_vio_stops():
    fusion = (SENSORS / "src/fixed_lag_eskf_component.cpp").read_text(encoding="utf-8")

    assert "body.linear_velocity_valid = vio_age <= inertial_horizon_s_" in fusion
    assert "body.position_estimated = estimated" in fusion
    assert 'return source + " (External IMU unavailable)";' in fusion
    assert "IMU calibrating" not in fusion
    assert 'source + " (gyro-only)"' in fusion
    assert "const bool imu_stale" in fusion
    assert "measurement_stamp > filter_.state().stamp_ns" in fusion
    assert "std::deque<VioMeasurement> vio_measurements_" in fusion
    assert "const bool accepted = replay_from(*index, measurement_stamp)" in fusion
    assert "record_accepted_vio(accepted, measurement_stamp, arrival_ns)" in fusion
    assert "measurement_stamp <= vio_freshness_.stamp_ns" in fusion
    assert "odom.header.stamp = rclcpp::Time(state.stamp_ns, RCL_ROS_TIME)" in fusion


def test_localization_status_exposes_rate_age_innovation_and_covariance():
    interface = (
        ROOT / "ros_ws/src/robotcore_interfaces/msg/LocalizationStatus.msg"
    ).read_text(encoding="utf-8")
    cmake = (
        ROOT / "ros_ws/src/robotcore_interfaces/CMakeLists.txt"
    ).read_text(encoding="utf-8")
    logger = (
        ROOT / "ros_ws/src/robotcore_runtime/robotcore_runtime/run_logger.py"
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
