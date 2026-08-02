"""Launch the real Jetson/A-board edge graph."""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    run_root = str(Path.cwd() / "data" / "robotcore_runs")

    return LaunchDescription(
        [
            # The 1a86 USB serial endpoint carries the A-board protocol: its
            # 115200-baud stream contains FF FB PWM feedback and FF F8 board
            # telemetry frames.  Bind by identity rather than tty numbering.
            DeclareLaunchArgument(
                "serial_port",
                default_value="/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B7A033320-if00",
            ),
            DeclareLaunchArgument("baud", default_value="115200"),
            DeclareLaunchArgument("zed_imu_topic", default_value="/zedx/zed_node/imu/data"),
            DeclareLaunchArgument("imu_raw_topic", default_value="/hardware/aboard_imu_raw"),
            DeclareLaunchArgument("external_imu_topic", default_value="/sensors/external_imu"),
            DeclareLaunchArgument("enable_external_imu", default_value="true"),
            DeclareLaunchArgument("depth_input_topic", default_value="/hardware/aboard_depth_m"),
            DeclareLaunchArgument("altitude_input_topic", default_value=""),
            DeclareLaunchArgument("front_camera_raw_topic", default_value="/zedx/zed_node/rgb/color/rect/image"),
            DeclareLaunchArgument(
                "front_camera_compressed_topic",
                default_value="/zedx/zed_node/rgb/color/rect/image/compressed",
            ),
            DeclareLaunchArgument(
                "front_camera_info_topic", default_value="/zedx/zed_node/rgb/color/rect/camera_info"
            ),
            DeclareLaunchArgument("enable_apriltag_localization", default_value="true"),
            DeclareLaunchArgument(
                "apriltag_detections_topic",
                default_value="/localization/apriltag/detections",
            ),
            # AprilTag calibrates map->odom. ZED VIO already fuses the camera
            # IMU and continuously propagates odom->base_link between tags.
            DeclareLaunchArgument("enable_tag_vio_alignment", default_value="true"),
            DeclareLaunchArgument("enable_fixed_rate_state_estimator", default_value="true"),
            DeclareLaunchArgument("enable_zed_visual_odometry", default_value="true"),
            DeclareLaunchArgument("zed_odometry_topic", default_value="/zedx/zed_node/odom"),
            DeclareLaunchArgument("zed_workspace", default_value="/home/nvidia/ros2_ws"),
            DeclareLaunchArgument(
                "zed_params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("eup_sensors"), "config", "zedx_minimal_open.yaml"]
                ),
            ),
            DeclareLaunchArgument("zed_serial_number", default_value="50649148"),
            DeclareLaunchArgument("zed_camera_id", default_value="-1"),
            DeclareLaunchArgument(
                "external_imu_config",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("eup_sensors"), "config", "external_imu.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "state_estimator_config",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("eup_sensors"),
                        "config",
                        "tag_vio_external_imu_ekf.yaml",
                    ]
                ),
            ),
            # Publish the quality-gated PnP observation directly.  Temporal
            # smoothing adds pose lag while the vehicle is moving; retain it
            # only when a deployment has measured a jitter problem.
            DeclareLaunchArgument("apriltag_pose_filter_time_constant_s", default_value="0.0"),
            # Zero disables gap-triggered filter resets.
            DeclareLaunchArgument("apriltag_pose_filter_reset_after_s", default_value="0.0"),
            # The alignment node publishes the fused map pose. Set this true
            # only for raw-tag TF diagnosis; it otherwise creates a competing
            # map->base_link transform.
            DeclareLaunchArgument("apriltag_publish_tf", default_value="false"),
            DeclareLaunchArgument(
                "apriltag_detected_count_topic",
                default_value="/localization/apriltag/detected_count",
            ),
            DeclareLaunchArgument("apriltag_max_tags", default_value="24"),
            DeclareLaunchArgument(
                "apriltag_tag_map_file",
                # The host manager and the separate web UI use this one
                # deployed map.  Keep it out of the installed package so a
                # web edit and a localisation reload always refer to exactly
                # the same file.
                default_value="/etc/robotcore/apriltag_map.json",
            ),
            DeclareLaunchArgument("web_host", default_value="0.0.0.0"),
            DeclareLaunchArgument("web_port", default_value="8080"),
            DeclareLaunchArgument("host_manager_socket", default_value=""),
            # Keep the historical all-in-one launch working for development.
            # Production systemd units set this false and run eup_ui as the
            # separate, least-privileged control-interface.service.
            DeclareLaunchArgument("enable_web_ui", default_value="true"),
            # Constrain every source (web, policy, and gamepad) to
            # 1400–1600 us around a 1500 us neutral command.
            DeclareLaunchArgument("manual_thruster_span_us", default_value="100"),
            DeclareLaunchArgument("manual_thruster_channel_offset", default_value="8"),
            DeclareLaunchArgument("aboard_poll_rate_hz", default_value="60.0"),
            # The ESC safety heartbeat is intentionally independent of the
            # 60 Hz RL action rate and remains at 100 Hz.
            DeclareLaunchArgument("thruster_command_write_hz", default_value="100.0"),
            DeclareLaunchArgument("thruster_command_timeout_ms", default_value="150"),
            DeclareLaunchArgument(
                "pool_control_config",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("eup_control"), "config", "real_pool_safety.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "pid_config_path",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("eup_control"), "config", "real_pool_pid.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "thruster_config_path",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("eup_control"), "config", "real_pool_thrusters.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "scenario_config_path",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("eup_runtime"), "config", "tracking_scenarios.yaml"]
                ),
            ),
            SetEnvironmentVariable(name="ROBOTCORE_RUN_ROOT", value=run_root),
            Node(
                package="eup_sensors",
                executable="sensor_fusion_node",
                name="sensor_fusion",
                output="screen",
                parameters=[
                    {
                        "depth_input_topic": LaunchConfiguration("depth_input_topic"),
                        "altitude_input_topic": LaunchConfiguration("altitude_input_topic"),
                        "fused_odometry_topic": "/localization/fused_odom",
                        "zed_odometry_topic": "/localization/zed_odom",
                        "localization_status_topic": "/localization/status",
                        "apriltag_pose_status_topic": "/localization/apriltag/pose_status",
                        "detected_tag_count_topic": LaunchConfiguration(
                            "apriltag_detected_count_topic"
                        ),
                        "tag_fused_sync_tolerance_s": 0.05,
                        "tag_vio_disagreement_deg": 10.0,
                        "degraded_localization_pose_topic": "/localization/apriltag_pose_degraded",
                    }
                ],
            ),
            Node(
                package="eup_sensors",
                executable="vehicle_frames_node",
                name="vehicle_frames",
                output="screen",
                parameters=[
                    LaunchConfiguration("external_imu_config"),
                    {
                        "base_frame": "base_link",
                        "imu_frame": "aboard_imu_link",
                        "camera_optical_frame": "front_camera_optical_frame",
                        "base_to_camera_translation_m": [0.236, 0.027, 0.016],
                        "camera_link_frame": "zedx_camera_link",
                    }
                ],
            ),
            Node(
                package="eup_sensors",
                executable="imu_conditioning_node",
                name="imu_conditioning",
                output="screen",
                condition=IfCondition(LaunchConfiguration("enable_external_imu")),
                parameters=[
                    LaunchConfiguration("external_imu_config"),
                    {
                        "input_topic": LaunchConfiguration("imu_raw_topic"),
                        "output_topic": LaunchConfiguration("external_imu_topic"),
                    },
                ],
            ),
            Node(
                package="eup_runtime",
                executable="zed_camera_launcher",
                name="zed_camera_launcher",
                output="screen",
                parameters=[
                    {
                        "zed_workspace": LaunchConfiguration("zed_workspace"),
                        "zed_params_file": LaunchConfiguration("zed_params_file"),
                        "zed_serial_number": ParameterValue(
                            LaunchConfiguration("zed_serial_number"), value_type=str
                        ),
                        "zed_camera_id": ParameterValue(
                            LaunchConfiguration("zed_camera_id"), value_type=str
                        ),
                    }
                ],
            ),
            # Isaac ROS owns only the image-space detector. Its single-size
            # pose and TF outputs are not authoritative because the managed
            # map contains both 0.4 m and 0.2 m Tags. The downstream map node
            # consumes ID/corners and performs one joint, per-Tag-size PnP.
            ComposableNodeContainer(
                package="rclcpp_components",
                executable="component_container_mt",
                name="apriltag_cuda_container",
                namespace="",
                output="screen",
                condition=IfCondition(LaunchConfiguration("enable_apriltag_localization")),
                composable_node_descriptions=[
                    ComposableNode(
                        package="isaac_ros_apriltag",
                        plugin="nvidia::isaac_ros::apriltag::AprilTagNode",
                        name="apriltag_cuda_detector",
                        parameters=[
                            {
                                "backends": "CUDA",
                                "tag_family": "tag36h11",
                                # Ignored by map localisation. It is needed by
                                # Isaac's non-authoritative raw pose output.
                                "size": 0.4,
                                "max_tags": ParameterValue(
                                    LaunchConfiguration("apriltag_max_tags"),
                                    value_type=int,
                                ),
                                "tile_size": 4,
                            }
                        ],
                        remappings=[
                            # Managed NITROS negotiates the /nitros endpoint
                            # from this base topic name automatically.
                            ("image", LaunchConfiguration("front_camera_raw_topic")),
                            ("camera_info", LaunchConfiguration("front_camera_info_topic")),
                            (
                                "tag_detections",
                                LaunchConfiguration("apriltag_detections_topic"),
                            ),
                            # Never let the detector's one-size tag poses join
                            # the production TF tree.
                            ("tf", "/localization/apriltag/raw_tf"),
                        ],
                    )
                ],
            ),
            Node(
                package="eup_sensors",
                executable="apriltag_localization_node",
                name="apriltag_localization",
                output="screen",
                condition=IfCondition(LaunchConfiguration("enable_apriltag_localization")),
                parameters=[
                    {
                        "camera_info_topic": LaunchConfiguration("front_camera_info_topic"),
                        "detections_topic": LaunchConfiguration("apriltag_detections_topic"),
                        "tag_map_file": LaunchConfiguration("apriltag_tag_map_file"),
                        "detected_count_topic": LaunchConfiguration("apriltag_detected_count_topic"),
                        "pose_status_topic": "/localization/apriltag/pose_status",
                        "tag_size_m": 0.130,
                        "pose_filter_time_constant_s": ParameterValue(
                            LaunchConfiguration("apriltag_pose_filter_time_constant_s"), value_type=float
                        ),
                        "pose_filter_reset_after_s": ParameterValue(
                            LaunchConfiguration("apriltag_pose_filter_reset_after_s"), value_type=float
                        ),
                        "base_frame": "base_link",
                        "map_frame": "map",
                        "minimum_pose_tag_count": 3,
                        "minimum_inlier_corners_per_tag": 3,
                        "degraded_pose_topic": "/localization/apriltag_pose_degraded",
                        "aligned_vio_odometry_topic": "/localization/aligned_vio_odom",
                        "enable_degraded_two_tag_pose": True,
                        "degraded_two_tag_inlier_corners_per_tag": 4,
                        "degraded_two_tag_max_full_rms_px": 3.0,
                        "degraded_two_tag_vio_sync_tolerance_s": 0.05,
                        "degraded_two_tag_vio_max_translation_m": 0.20,
                        "degraded_two_tag_vio_max_angle_deg": 10.0,
                        "tag_map_position_uncertainty_m": 0.05,
                        "enforce_cuboid_pool_geometry": True,
                        "pool_length_m": 5.42,
                        "pool_width_m": 3.73,
                        "pool_surface_tolerance_m": 0.02,
                        "pool_orientation_tolerance_deg": 2.0,
                        "max_reprojection_rms_px": 3.0,
                        "max_translation_jump_m": 0.05,
                        "multi_tag_position_stddev_m": 0.05,
                        "enforce_observation_gates": True,
                        "enforce_transition_gate": True,
                        "publish_tf": ParameterValue(
                            LaunchConfiguration("apriltag_publish_tf"), value_type=bool
                        ),
                        "base_to_camera_translation_m": [0.236, 0.027, 0.016],
                    }
                ],
            ),
            # Adapt ZED's local odometry from camera_link into base_link,
            # retaining its local odom-frame pose for map alignment.
            Node(
                package="eup_sensors",
                executable="zed_odometry_adapter_node",
                name="zed_odometry_adapter",
                output="screen",
                condition=IfCondition(LaunchConfiguration("enable_zed_visual_odometry")),
                parameters=[
                    {
                        "input_topic": LaunchConfiguration("zed_odometry_topic"),
                        "output_topic": "/localization/zed_odom",
                        "base_frame": "base_link",
                    }
                ],
            ),
            Node(
                package="eup_sensors",
                executable="tag_vio_alignment_node",
                name="tag_vio_alignment",
                output="screen",
                condition=IfCondition(LaunchConfiguration("enable_tag_vio_alignment")),
                parameters=[
                    {
                        "tag_pose_topic": "/localization/apriltag_pose",
                        "vio_odometry_topic": "/localization/zed_odom",
                        "output_odometry_topic": "/localization/aligned_vio_odom",
                        "map_frame": "map",
                        "base_frame": "base_link",
                        "alignment_correction_alpha": 0.25,
                        "alignment_confirm_frames": 4,
                        "alignment_candidate_max_spread_m": 0.20,
                        "alignment_candidate_max_angle_deg": 12.0,
                        "alignment_recalibration_threshold_m": 0.05,
                        "alignment_recalibration_threshold_deg": 2.0,
                        "tag_vio_sync_tolerance_s": 0.05,
                        "alignment_max_correction_m": 0.75,
                    }
                ],
            ),
            # The alignment output contains the latest absolute Tag correction,
            # ZED VIO pose, and ZED linear velocity. The independent UART8 gyro
            # propagates attitude between those visual updates. robot_localization
            # owns the canonical fixed-rate /localization/fused_odom stream.
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="localization_ekf",
                output="screen",
                condition=IfCondition(
                    LaunchConfiguration("enable_fixed_rate_state_estimator")
                ),
                parameters=[
                    LaunchConfiguration("state_estimator_config"),
                    {
                        "imu0": LaunchConfiguration("external_imu_topic"),
                    },
                ],
                remappings=[("odometry/filtered", "/localization/fused_odom")],
            ),
            Node(
                package="eup_runtime",
                executable="trajectory_command_node",
                name="trajectory_command",
                output="screen",
                parameters=[LaunchConfiguration("pool_control_config")],
            ),
            Node(
                package="eup_runtime",
                executable="tracking_monitor_node",
                name="tracking_monitor",
                output="screen",
                parameters=[{"publish_rate_hz": 20.0}],
            ),
            Node(
                package="eup_control",
                executable="pid_controller",
                name="pid_controller",
                output="screen",
                parameters=[
                    {
                        "pid_config_path": LaunchConfiguration("pid_config_path"),
                        "thruster_config_path": LaunchConfiguration(
                            "thruster_config_path"
                        ),
                        "control_rate_hz": 60.0,
                    }
                ],
            ),
            Node(
                package="eup_control",
                executable="command_authority",
                name="command_authority",
                output="screen",
                parameters=[LaunchConfiguration("pool_control_config")],
            ),
            Node(
                package="eup_runtime",
                executable="tracking_experiment_node",
                name="tracking_experiment",
                output="screen",
                parameters=[
                    {
                        "scenario_config_path": LaunchConfiguration(
                            "scenario_config_path"
                        )
                    }
                ],
            ),
            Node(
                package="eup_runtime",
                executable="safety_monitor",
                name="safety_monitor",
                output="screen",
            ),
            Node(
                package="eup_runtime",
                executable="blackboard",
                name="blackboard",
                output="screen",
            ),
            Node(
                package="eup_hardware",
                executable="aboard_bridge_node",
                name="aboard_bridge",
                output="screen",
                parameters=[
                    {
                        "serial_port": LaunchConfiguration("serial_port"),
                        "baud": ParameterValue(LaunchConfiguration("baud"), value_type=int),
                        "span_us": ParameterValue(
                            LaunchConfiguration("manual_thruster_span_us"),
                            value_type=int,
                        ),
                        "thruster_channel_offset": ParameterValue(
                            LaunchConfiguration("manual_thruster_channel_offset"),
                            value_type=int,
                        ),
                        "command_write_hz": ParameterValue(
                            LaunchConfiguration("thruster_command_write_hz"),
                            value_type=float,
                        ),
                        "poll_rate_hz": ParameterValue(
                            LaunchConfiguration("aboard_poll_rate_hz"),
                            value_type=float,
                        ),
                        "command_timeout_ms": ParameterValue(
                            LaunchConfiguration("thruster_command_timeout_ms"),
                            value_type=int,
                        ),
                        # UART8 is an independent high-quality IMU. Publish only
                        # valid frame-3 samples; the conditioning node estimates
                        # stationary gyro bias before the 60 Hz EKF consumes it.
                        "publish_imu": True,
                        "imu_topic": LaunchConfiguration("imu_raw_topic"),
                        "imu_frame_id": "aboard_imu_link",
                    }
                ],
            ),
            Node(
                package="eup_ui",
                executable="web_operator_server",
                name="web_operator",
                output="screen",
                condition=IfCondition(LaunchConfiguration("enable_web_ui")),
                parameters=[
                    {
                        "web_host": LaunchConfiguration("web_host"),
                        "web_port": LaunchConfiguration("web_port"),
                        "host_manager_socket": LaunchConfiguration("host_manager_socket"),
                        "apriltag_map_file": LaunchConfiguration("apriltag_tag_map_file"),
                        "apriltag_detected_count_topic": LaunchConfiguration(
                            "apriltag_detected_count_topic"
                        ),
                        "title": "RobotCore Operator",
                        "front_camera_compressed_topic": LaunchConfiguration("front_camera_compressed_topic"),
                        "imu_topic": LaunchConfiguration("zed_imu_topic"),
                        "manual_thruster_span_us": ParameterValue(
                            LaunchConfiguration("manual_thruster_span_us"),
                            value_type=int,
                        ),
                        "manual_thruster_channel_offset": ParameterValue(
                            LaunchConfiguration("manual_thruster_channel_offset"),
                            value_type=int,
                        ),
                        "manual_thruster_topic": "/control/candidates/manual",
                    }
                ],
            ),
        ]
    )
