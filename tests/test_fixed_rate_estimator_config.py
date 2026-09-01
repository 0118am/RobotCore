"""Static acceptance tests for the edge localisation pipeline."""

from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
SENSORS = ROOT / "ros_ws/src/robotcore_sensors"


def load_yaml(relative: str):
    return yaml.safe_load((SENSORS / relative).read_text(encoding="utf-8"))


def test_zed_camera_grabs_at_30_hz_and_caps_processing_at_15_hz():
    parameters = load_yaml("config/zedx_minimal_open.yaml")["/**"]["ros__parameters"]

    assert parameters["general"]["grab_frame_rate"] == 30
    assert parameters["general"]["grab_compute_capping_fps"] == 15.0
    assert parameters["general"]["pub_frame_rate"] == 15.0
    assert parameters["general"]["pub_resolution"] == "NATIVE"
    assert parameters["general"]["camera_max_reconnect"] == 5
    assert parameters["general"]["sdk_use_monotonic_clock"] is True
    assert parameters["debug"]["use_pub_timestamps"] is False
    assert parameters["general"]["grab_resolution"] == "SVGA"
    assert parameters["sensors"]["publish_imu"] is False
    assert parameters["depth"]["depth_mode"] == "NONE"
    assert parameters["pos_tracking"]["pos_tracking_enabled"] is True
    assert parameters["pos_tracking"]["pos_tracking_mode"] == "GEN_3"
    assert parameters["pos_tracking"]["imu_fusion"] is True
    assert parameters["pos_tracking"]["area_memory"] is False
    assert parameters["pos_tracking"]["reset_odom_with_loop_closure"] is False
    assert parameters["pos_tracking"]["publish_odom_pose"] is True
    assert parameters["pos_tracking"]["publish_pose_cov"] is True


def test_vio_tag_fusion_is_one_native_cpp_ekf():
    source = (SENSORS / "src/vio_tag_fusion_component.cpp").read_text()
    cmake = (SENSORS / "CMakeLists.txt").read_text()

    assert "VioTagFusionComponent" in source
    assert 'Node("ekf", options)' in source
    assert "map_from_odom_" in source
    assert "class VioTagEkf" in source
    assert "update_linear_velocity(" in source
    assert "update_orientation(" in source
    assert "update_angular_velocity(" in source
    assert '"vio_linear_velocity_stddev_floor_mps", 0.10' in source
    assert '"vio_linear_velocity_correction_limit_mps", 0.04' in source
    assert '"vio_linear_velocity_innovation_limit_mps", 0.25' in source
    assert "event.source == MeasurementSource::Tag" in source
    assert "position_state_only" in source
    assert "MeasurementResult replay(" in source
    assert "state_at(" in source
    assert "alignment_covariance_" in source
    assert "alignment_candidate_covariance(" in source
    assert "covariance_sum_with_unknown_correlation<6>(" in source
    assert "pose_covariance += alignment_jacobian" not in source
    assert "const auto candidate_covariance" not in source
    assert "0.5 * (state_.covariance + state_.covariance.transpose())" not in source
    assert "0.5 * (alignment_covariance_ + alignment_covariance_.transpose())" not in source
    assert "0.5 * (pose_covariance + pose_covariance.transpose())" not in source
    assert "symmetrized_covariance<6>(raw_state_pose_covariance)" in source
    assert "symmetrized_covariance<6>(raw_alignment_contribution)" in source
    assert "invalid localization covariance; resetting estimator" in source
    assert "SensorDataQoS().keep_last(1)" in source
    assert "enforce_covariance_floors" not in source
    assert "use_vio_velocity_covariance_fallback" not in source
    assert "vio_velocity_fallback_stddev_mps" not in source
    assert "sensor_msgs::msg::Imu" not in source
    assert "MeasurementSource::Imu" not in source
    assert "gain.template block<6, Size>(6, 0).setZero()" in source
    assert "gain.template block<6, Size>(0, 0).setZero()" in source
    apply_measurement = source[
        source.index("MeasurementResult apply_measurement"):
        source.index("void insert_measurement")
    ]
    assert apply_measurement.count("filter.update_orientation(") == 1
    assert apply_measurement.count("filter.update_angular_velocity(") == 1
    assert apply_measurement.count("filter.update_position(") == 1
    assert apply_measurement.count("filter.update_linear_velocity(") == 1
    assert "event.pose_covariance.block<3, 3>(0, 0), pose_gate, true" in apply_measurement
    assert "linear_velocity_correction_limit_mps_" in apply_measurement
    assert 'status.add(\n      "vio_linear_velocity_correction_limits"' in source
    assert 'status.add(\n      "vio_linear_velocity_rejections"' in source
    assert "external_imu" not in apply_measurement
    assert "-O3" in cmake
    assert "CMAKE_INTERPROCEDURAL_OPTIMIZATION" in cmake
    assert "ffast-math" not in cmake
    assert "robotcore_localization_core" not in cmake
    assert not (SENSORS / "include/robotcore_sensors/fixed_lag_eskf.hpp").exists()
    assert not (SENSORS / "src/fixed_lag_eskf.cpp").exists()
    assert not (SENSORS / "src/fixed_lag_eskf_component.cpp").exists()
    assert not (SENSORS / "src/zed_odometry_adapter_component.cpp").exists()
    assert not (SENSORS / "config/tag_vio_external_imu_ekf.yaml").exists()


def test_aboard_bridge_publishes_the_single_factory_calibrated_base_imu():
    bridge = (
        ROOT / "ros_ws/src/robotcore_hardware/src/aboard_bridge_node.cpp"
    ).read_text(encoding="utf-8")
    cmake = (SENSORS / "CMakeLists.txt").read_text(encoding="utf-8")

    assert '"/sensors/external_imu", rclcpp::SensorDataQoS().keep_last(1)' in bridge
    assert 'message.header.frame_id = "base_link"' in bridge
    assert "sample.gyro_rad_s" in bridge
    assert "sample.accel_m_s2" in bridge
    assert "sample.attitude_rpy_rad" in bridge
    assert "message.orientation.w" in bridge
    assert "imu_duplicate_or_backwards_" in bridge
    assert "/hardware/aboard_imu_raw" not in bridge
    assert "ImuConditionerComponent" not in cmake
    assert not (SENSORS / "src/imu_conditioner_component.cpp").exists()
    assert not (SENSORS / "include/robotcore_sensors/imu_conditioning.hpp").exists()
    assert not (SENSORS / "config/external_imu.yaml").exists()


def test_edge_launch_wires_one_canonical_fixed_rate_output():
    launch = (
        ROOT / "ros_ws/src/robotcore_bringup/launch/robotcore_edge_system.launch.py"
    ).read_text(encoding="utf-8")
    fusion_config = load_yaml("config/localization.yaml")["/ekf"]["ros__parameters"]

    assert "ImuConditionerComponent" not in launch
    assert "imu_raw_topic" not in launch
    assert "external_imu_topic" not in launch
    assert 'plugin="robotcore_sensors::VioTagFusionComponent"' in launch
    assert 'name="ekf"' in launch
    assert 'name="vio_tag_fusion"' not in launch
    assert "ZedOdometryAdapterComponent" not in launch
    assert "FixedLagEskfComponent" not in launch
    assert 'LaunchConfiguration("localization_config")' in launch
    assert fusion_config["vio_topic"] == "/zedx/zed_node/odom"
    assert "/localization/zed_odom" not in launch
    assert "sensor_msgs::msg::Imu" not in (
        SENSORS / "src/vio_tag_fusion_component.cpp"
    ).read_text(encoding="utf-8")
    assert '"use_vio"' not in launch
    assert fusion_config["vio_arrival_timeout_s"] == 0.80
    assert fusion_config["vio_prediction_horizon_s"] == 0.80
    assert fusion_config["vio_linear_velocity_stddev_floor_mps"] == 0.10
    assert fusion_config["vio_linear_velocity_correction_limit_mps"] == 0.04
    assert fusion_config["vio_linear_velocity_innovation_limit_mps"] == 0.25
    fusion_parameters = launch[
        launch.index('plugin="robotcore_sensors::VioTagFusionComponent"'):
        launch.index('extra_arguments=[{"use_intra_process_comms": True}]',
                     launch.index('plugin="robotcore_sensors::VioTagFusionComponent"'))
    ]
    assert '"imu_topic"' not in fusion_parameters
    assert "imu_arrival_timeout_s" not in fusion_parameters
    trajectory_parameters = launch[
        launch.index('executable="trajectory_command_node"'):
        launch.index('executable="tracking_monitor_node"')
    ]
    pid_parameters = launch[
        launch.index('executable="pid_controller"'):
        launch.index('executable="command_authority"')
    ]
    assert '"imu_topic": "/sensors/external_imu"' in trajectory_parameters
    assert '"imu_topic": "/sensors/external_imu"' in pid_parameters
    assert "use_vio_velocity_covariance_fallback" not in fusion_parameters
    assert "base_to_aboard_imu" not in launch
    assert fusion_config["tag_topic"] == "/localization/apriltag_pose"
    assert "constrain_vertical_acceleration" not in launch
    assert fusion_config["tag_fresh_s"] == 0.35
    assert '"tag_innovation_gate_m": 0.50' not in launch
    assert fusion_config["require_zed_tracking_ok"] is True
    assert fusion_config["output_rate_hz"] == 60.0
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
    tag_config = load_yaml("config/localization.yaml")["/apriltag_localization"][
        "ros__parameters"
    ]

    assert tag_config["minimum_pose_tag_count"] == 1
    assert tag_config["maximum_pose_tag_count"] == 3
    assert tag_config["maximum_tag_edge_ratio"] == 2.5
    assert "assess_tag_image_quality(" in localizer
    assert "left.quality.score > right.quality.score" in localizer
    assert "candidates.resize" in localizer
    assert "cv::SOLVEPNP_IPPE" in localizer
    assert "cv::solvePnPRansac" in localizer
    assert "dual_tag_position_stddev_m" in localizer
    assert "single_tag_position_stddev_m" in localizer


def test_relocalization_uses_a_bounded_mutually_consistent_candidate_window():
    fusion = (SENSORS / "src/vio_tag_fusion_component.cpp").read_text(
        encoding="utf-8"
    )
    parameters = load_yaml("config/localization.yaml")["/ekf"]["ros__parameters"]

    assert parameters["alignment_candidate_count"] == 4
    assert parameters["alignment_candidate_window_s"] == 8.0
    assert parameters["alignment_translation_tolerance_m"] == 0.20
    assert parameters["alignment_rotation_tolerance_deg"] == 12.0
    assert "tightest_consistent_pose_cluster(" in fusion
    assert "alignment_candidate_cluster_size" in fusion
    assert "AprilTag map alignment established" in fusion
    assert "stamp - alignment_candidates_.back().stamp_ns > 500000000LL" not in fusion


def test_estimator_rate_probe_drives_current_cpp_inputs():
    probe = (ROOT / "scripts/robotcore_estimator_rate_check.py").read_text(
        encoding="utf-8"
    )

    assert 'Odometry, "/zedx/zed_node/odom", 1' in probe
    assert 'Imu, "/sensors/external_imu", 50' not in probe
    assert 'PosTrackStatus, "/zedx/zed_node/pose/status", 1' in probe
    assert "publish_imu" not in probe
    assert "self.create_timer(1.0 / 30.0, self.publish_vio)" in probe
    assert 'AprilTagPoseEstimate, "/localization/apriltag_pose", 1' in probe
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
    assert '"ZED VIO EKF after Tag loss"' in fusion
    assert "vio_arrival_age_s <= vio_arrival_timeout_s_" in fusion
    assert "vio_measurement_age_s <= vio_prediction_horizon_s_" in fusion
    assert "tag_arrival_age_s <= tag_fresh_s_" in fusion
    assert "create_wall_timer" in fusion
    assert "1.0 / std::max(1.0, output_hz)" in fusion
    assert "body_pub_->publish(body)" in fusion
    assert "IMU calibrating" not in fusion
    assert "sensor_msgs::msg::Imu" not in fusion
    assert "output_filter.propagate_to(now_ns)" in fusion
    assert "update_linear_velocity(" in fusion
    assert "filter.update_orientation(" in fusion
    assert "filter.update_angular_velocity(" in fusion
    assert "state_at(tag_stamp_ns)" in fusion
    assert "continuous_rejection_reanchor_required" not in fusion
    assert "MeasurementResult replay(" in fusion
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
        "body_state_rate_hz",
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

    body_publish = fusion.index("body_pub_->publish(body)")
    body_arrival = fusion.index("body_state_arrivals_.push_back(now_ns)")
    assert body_publish < body_arrival
    assert "arrival_rate_hz(" in fusion
    assert '"AprilTag/ZED VIO EKF estimating in map"' in fusion
    assert 'status.absolute_fix_valid = absolute_valid' in fusion
