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
    assert parameters["general"]["sdk_use_monotonic_clock"] is True
    assert parameters["debug"]["use_pub_timestamps"] is False
    assert parameters["general"]["grab_resolution"] == "SVGA"
    assert parameters["general"]["pub_downscale_factor"] == 1.0
    assert parameters["sensors"]["publish_imu"] is False
    assert parameters["depth"]["depth_mode"] == "NEURAL_LIGHT"
    assert parameters["pos_tracking"]["pos_tracking_enabled"] is True
    assert parameters["pos_tracking"]["pos_tracking_mode"] == "GEN_3"
    assert parameters["pos_tracking"]["imu_fusion"] is True
    assert parameters["pos_tracking"]["area_memory"] is False
    assert parameters["pos_tracking"]["reset_odom_with_loop_closure"] is False
    assert parameters["pos_tracking"]["publish_odom_pose"] is True


def test_vio_tag_fusion_is_native_cpp_and_old_eskf_is_removed():
    source = (SENSORS / "src/vio_tag_fusion_component.cpp").read_text()
    cmake = (SENSORS / "CMakeLists.txt").read_text()

    assert "VioTagFusionComponent" in source
    assert "map_from_odom_" in source
    assert "update_alignment(" in source
    assert "alignment_covariance_" in source
    assert "SensorDataQoS().keep_last(1)" in source
    assert "-O3" in cmake
    assert "CMAKE_INTERPROCEDURAL_OPTIMIZATION" in cmake
    assert "ffast-math" not in cmake
    assert "FixedLagEskf" not in cmake
    assert not (SENSORS / "include/robotcore_sensors/fixed_lag_eskf.hpp").exists()
    assert not (SENSORS / "src/fixed_lag_eskf.cpp").exists()
    assert not (SENSORS / "src/fixed_lag_eskf_component.cpp").exists()
    assert not (SENSORS / "src/zed_odometry_adapter_component.cpp").exists()
    assert not (SENSORS / "config/tag_vio_external_imu_ekf.yaml").exists()


def test_external_imu_trusts_factory_calibration_and_filters_by_timestamp():
    config = load_yaml("config/external_imu.yaml")
    parameters = config["imu_conditioning"]["ros__parameters"]
    conditioner = (SENSORS / "src/imu_conditioner_component.cpp").read_text()

    assert parameters["input_topic"] == "/hardware/aboard_imu_raw"
    assert parameters["output_topic"] == "/sensors/external_imu"
    assert "ui_output_topic" not in parameters
    assert "ui_publish_rate_hz" not in parameters
    assert "fusion_output_topic" not in parameters
    assert "zeroed_output_topic" not in parameters
    assert parameters["gyro_low_pass_cutoff_hz"] == 20.0
    assert parameters["accel_low_pass_cutoff_hz"] == 15.0
    assert parameters["filter_reset_gap_s"] == 0.2
    assert "zero_vertical_acceleration" not in parameters
    assert len(parameters["base_to_imu_rpy_rad"]) == 3
    assert parameters["base_frame_id"] == "base_link"
    assert "output_pub_->publish(output)" in conditioner
    assert '"/ui/external_imu"' not in conditioner
    assert "fusion_output_pub_" not in conditioner
    assert "zeroed_output_pub_" not in conditioner
    assert "constrain_vertical_acceleration" not in conditioner
    assert "output.header.frame_id = base_frame_id_" in conditioner
    assert "rotate_imu_vector_to_base(raw_gyro, base_from_imu_)" in conditioner
    assert "gyro_filter_->update(base_gyro, stamp)" in conditioner
    for removed in (
        "calibration_file", "trust_device_factory", "restart_calibration",
        "runtime_gyro_bias", "runtime_accel_residual", "gyro-only fallback",
    ):
        assert removed not in conditioner


def test_edge_launch_wires_one_canonical_fixed_rate_output():
    launch = (
        ROOT / "ros_ws/src/robotcore_bringup/launch/robotcore_edge_system.launch.py"
    ).read_text(encoding="utf-8")

    assert 'plugin="robotcore_sensors::ImuConditionerComponent"' in launch
    assert 'plugin="robotcore_sensors::VioTagFusionComponent"' in launch
    assert "ZedOdometryAdapterComponent" not in launch
    assert "FixedLagEskfComponent" not in launch
    assert '"vio_topic": "/zedx/zed_node/odom"' in launch
    assert "/localization/zed_odom" not in launch
    assert "sensor_msgs::msg::Imu" not in (
        SENSORS / "src/vio_tag_fusion_component.cpp"
    ).read_text(encoding="utf-8")
    assert '"use_vio"' not in launch
    assert '"vio_arrival_timeout_s": 0.30' in launch
    assert '"vio_prediction_horizon_s": 0.50' in launch
    assert "external_imu_fusion_topic" not in launch
    assert "base_to_aboard_imu" not in launch
    assert '"tag_topic": "/localization/apriltag_pose"' in launch
    assert "constrain_vertical_acceleration" not in launch
    assert '"tag_fresh_s": 0.35' in launch
    assert '"tag_innovation_gate_m": 0.50' in launch
    assert '"output_rate_hz": 60.0' in launch
    assert launch.count('"/localization/fused_odom"') == 0
    assert "pressure_depth_odometry_node" not in launch
    assert "/localization/pressure_depth_odom" not in launch
    assert 'package="robot_localization"' not in launch
    assert "sensor_fusion_node" not in launch
    assert 'DeclareLaunchArgument("enable_pool_tracking", default_value="true")' in launch
    assert launch.count(
        'condition=IfCondition(LaunchConfiguration("enable_pool_tracking"))'
    ) == 5


def test_apriltag_pose_uses_quality_ranked_one_two_or_best_three_observations():
    localizer = (SENSORS / "src/apriltag_map_localizer_component.cpp").read_text(
        encoding="utf-8"
    )
    launch = (
        ROOT / "ros_ws/src/robotcore_bringup/launch/robotcore_edge_system.launch.py"
    ).read_text(encoding="utf-8")

    assert '"minimum_pose_tag_count": 1' in launch
    assert '"maximum_pose_tag_count": 3' in launch
    assert '"maximum_tag_edge_ratio": 2.5' in launch
    assert "assess_tag_image_quality(" in localizer
    assert "left.quality.score > right.quality.score" in localizer
    assert "candidates.resize" in localizer
    assert "cv::SOLVEPNP_IPPE" in localizer
    assert "cv::solvePnPRansac" in localizer
    assert "dual_tag_position_stddev_m" in localizer
    assert "single_tag_position_stddev_m" in localizer


def test_estimator_rate_probe_drives_current_cpp_inputs():
    probe = (ROOT / "scripts/robotcore_estimator_rate_check.py").read_text(
        encoding="utf-8"
    )

    assert 'Odometry, "/zedx/zed_node/odom", 10' in probe
    assert 'Imu, "/sensors/external_imu", 50' not in probe
    assert "publish_imu" not in probe
    assert "self.create_timer(1.0 / 30.0, self.publish_vio)" in probe
    assert 'AprilTagPoseEstimate, "/localization/apriltag_pose", 10' in probe
    assert "message.map_generation = 1" in probe
    assert "message.pose_valid = True" in probe
    assert "self.create_timer(1.0 / 30.0, self.publish_tag_anchor)" in probe
    assert "if elapsed_s > 2.0" in probe
    assert "tag_position_error" in probe
    assert 'frame == "map"' in probe
    assert "message.state_valid" in probe
    assert "message.position_estimated" in probe
    assert "message.linear_velocity_valid" in probe
    assert 'Odometry, "/localization/aligned_vio_odom"' not in probe


def test_body_state_uses_vio_for_local_tracking_and_tag_for_absolute_validity():
    fusion = (SENSORS / "src/vio_tag_fusion_component.cpp").read_text(encoding="utf-8")

    assert "body.linear_velocity_valid = vio_usable" in fusion
    assert "body.position_estimated = estimated" in fusion
    assert '"ZED VIO after Tag loss"' in fusion
    assert "arrival_age_s <= vio_arrival_timeout_s_" in fusion
    assert "measurement_age_s <= vio_prediction_horizon_s_" in fusion
    assert "tag_arrival_age_s <= tag_fresh_s_" in fusion
    assert "create_wall_timer" in fusion
    assert "1.0 / std::max(1.0, output_hz)" in fusion
    assert "body_pub_->publish(body)" in fusion
    assert "IMU calibrating" not in fusion
    assert "sensor_msgs::msg::Imu" not in fusion
    assert "filter_.propagate(" not in fusion
    assert "predicted_odom_from_base(" in fusion
    assert "update_alignment(" in fusion
    assert "sample_at(tag_stamp_ns)" in fusion
    assert "continuous_rejection_reanchor_required" not in fusion
    assert "replay_from(" not in fusion
    assert "fused_pub_ = create_publisher<nav_msgs::msg::Odometry>(" in fusion
    assert "body_pub_ = create_publisher<robotcore_interfaces::msg::BodyState>(" in fusion
    assert '"/ui/body_state"' not in fusion
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


def test_estimator_rates_and_diagnostics_follow_live_freshness():
    fusion = (SENSORS / "src/vio_tag_fusion_component.cpp").read_text(
        encoding="utf-8"
    )

    fused_publish = fusion.index("fused_pub_->publish(odometry)")
    fused_arrival = fusion.index("fused_arrivals_.push_back(now_ns)")
    assert fused_publish < fused_arrival
    assert "arrival_rate_hz(" in fusion
    assert 'summary = "local VIO estimate after Tag loss"' in fusion
    assert 'status.add("absolute_fix_valid", absolute_valid)' in fusion
