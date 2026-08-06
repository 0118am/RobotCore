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
    conditioner = (SENSORS / "src/imu_conditioner_component.cpp").read_text()

    assert parameters["input_topic"] == "/hardware/aboard_imu_raw"
    assert parameters["output_topic"] == "/sensors/external_imu"
    assert "fusion_output_topic" not in parameters
    assert "zeroed_output_topic" not in parameters
    assert parameters["status_topic"] == "/localization/external_imu_ready"
    assert parameters["calibration_sample_count"] == 250
    assert parameters["calibration_timeout_s"] == 10.0
    assert parameters["input_freshness_timeout_s"] == 0.5
    assert parameters["calibration_file"] == "/etc/robotcore/external_imu_calibration.yaml"
    assert "restart_calibration();" not in conditioner.split("private:", 1)[0]
    assert len(parameters["base_to_imu_rpy_rad"]) == 3
    assert parameters["base_frame_id"] == "base_link"

    assert "IMU calibration rejected: no raw IMU telemetry received" in conditioner
    assert "IMU calibration rejected: raw IMU telemetry is stale" in conditioner
    assert "IMU calibration already in progress" in conditioner
    assert "gyro_stationary && accel_stationary" in conditioner
    assert "runtime_accel_residual_ = measured_accel - expected" in conditioner
    assert "accel - runtime_accel_residual" in conditioner
    assert "previous correction retained" in conditioner
    assert "output_pub_->publish(output)" in conditioner
    assert "fusion_output_pub_" not in conditioner
    assert "zeroed_output_pub_" not in conditioner
    assert "base_accel.z() - 9.80665" not in conditioner
    assert "output.header.frame_id = base_frame_id_" in conditioner
    assert "rotate_imu_vector_to_base(corrected_gyro, base_from_imu_)" in conditioner
    assert "persistent_calibration_valid_, runtime_accel_residual_.norm()" in conditioner
    assert "Loaded external IMU calibration for this service start" in conditioner
    assert "expected_sensor_serial" not in conditioner
    assert "expected_config_hash" not in conditioner
    assert "calibration_hash" not in conditioner
    assert "calibration_sensor_serial" not in conditioner
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
    assert '"imu_topic": LaunchConfiguration("external_imu_topic")' in launch
    assert "external_imu_fusion_topic" not in launch
    assert "base_to_aboard_imu" not in launch
    assert '"tag_topic": "/localization/apriltag_pose"' in launch
    assert '"output_rate_hz": 60.0' in launch
    assert launch.count('"/localization/fused_odom"') == 0
    assert "pressure_depth_odometry_node" not in launch
    assert "/localization/pressure_depth_odom" not in launch
    assert 'package="robot_localization"' not in launch
    assert "sensor_fusion_node" not in launch
    assert 'DeclareLaunchArgument("enable_pool_tracking", default_value="false")' in launch
    assert launch.count(
        'condition=IfCondition(LaunchConfiguration("enable_pool_tracking"))'
    ) == 4


def test_estimator_rate_probe_drives_current_cpp_inputs():
    probe = (ROOT / "scripts/robotcore_estimator_rate_check.py").read_text(
        encoding="utf-8"
    )

    assert 'Odometry, "/localization/zed_odom"' in probe
    assert 'Imu, "/sensors/external_imu", 50' in probe
    assert "self.create_timer(1.0 / 100.0, self.publish_imu)" in probe
    assert 'AprilTagPoseEstimate, "/localization/apriltag_pose", 10' in probe
    assert "message.map_generation = 1" in probe
    assert "message.pose_valid = True" in probe
    assert "self.create_timer(1.0 / 10.0, self.publish_tag_anchor)" in probe
    assert 'message.header.frame_id = "odom"' in probe
    assert "message.linear_acceleration.z = 9.80665" in probe
    assert "tag_position_correction" in probe
    assert 'frame == "map"' in probe
    assert "message.state_valid" in probe
    assert 'Odometry, "/localization/aligned_vio_odom"' not in probe


def test_body_state_marks_velocity_stale_when_zed_vio_stops():
    fusion = (SENSORS / "src/fixed_lag_eskf_component.cpp").read_text(encoding="utf-8")

    assert "body.linear_velocity_valid = vio_age <= inertial_horizon_s_" in fusion
    assert "body.position_estimated = estimated" in fusion
    assert 'return source + " (External IMU unavailable)";' in fusion
    assert "IMU calibrating" not in fusion
    assert 'source + " (gyro-only)"' in fusion
    assert "const bool imu_stale" in fusion
    assert "vio_bridge_required(" in fusion
    assert "measurement_stamp, filter_.state().stamp_ns" in fusion
    assert "std::deque<PoseMeasurement> measurements_" in fusion
    assert "event.source = MeasurementSource::Vio" in fusion
    assert "event.source = MeasurementSource::Tag" in fusion
    assert "const bool accepted = replay_from(*index, event.id)" in fusion
    assert "record_accepted_vio(accepted, measurement_stamp, arrival_ns)" in fusion
    assert "measurement_stamp <= vio_freshness_.stamp_ns" in fusion
    assert "odom.header.stamp = rclcpp::Time(state.stamp_ns, RCL_ROS_TIME)" in fusion
    assert 'create_publisher<nav_msgs::msg::Odometry>("/localization/fused_odom"' in fusion
    assert "aligned_pub_" not in fusion


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
    assert '"msg/AprilTagPoseEstimate.msg"' in cmake
    assert '"msg/AprilTagPoseStatus.msg"' not in cmake
