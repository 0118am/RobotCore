"""Launch the real Jetson/A-board edge graph."""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
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
            DeclareLaunchArgument("depth_input_topic", default_value="/hardware/aboard_depth_m"),
            DeclareLaunchArgument("altitude_input_topic", default_value=""),
            DeclareLaunchArgument("front_camera_raw_topic", default_value="/zedx/zed_node/rgb/color/rect/image"),
            DeclareLaunchArgument(
                "front_camera_tag_overlay_topic",
                default_value="/localization/apriltag/debug_image/compressed",
            ),
            DeclareLaunchArgument(
                "front_camera_compressed_topic",
                default_value="/zedx/zed_node/rgb/color/rect/image/compressed",
            ),
            DeclareLaunchArgument("camera_source_lock", default_value="true"),
            DeclareLaunchArgument(
                "front_camera_info_topic", default_value="/zedx/zed_node/rgb/color/rect/camera_info"
            ),
            DeclareLaunchArgument("enable_apriltag_localization", default_value="true"),
            # AprilTag calibrates map->odom. ZED VIO already fuses the camera
            # IMU and continuously propagates odom->base_link between tags.
            DeclareLaunchArgument("enable_tag_vio_alignment", default_value="true"),
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
            DeclareLaunchArgument("thruster_command_timeout_ms", default_value="500"),
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
                    {
                        "base_frame": "base_link",
                        "camera_optical_frame": "front_camera_optical_frame",
                        "base_to_camera_translation_m": [0.236, 0.027, 0.016],
                        "camera_link_frame": "zedx_camera_link",
                    }
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
            Node(
                package="eup_sensors",
                executable="apriltag_localization_node",
                name="apriltag_localization",
                output="screen",
                condition=IfCondition(LaunchConfiguration("enable_apriltag_localization")),
                parameters=[
                    {
                        "image_topic": LaunchConfiguration("front_camera_raw_topic"),
                        "camera_info_topic": LaunchConfiguration("front_camera_info_topic"),
                        "debug_image_topic": LaunchConfiguration("front_camera_tag_overlay_topic"),
                        "tag_map_file": LaunchConfiguration("apriltag_tag_map_file"),
                        "detected_count_topic": LaunchConfiguration("apriltag_detected_count_topic"),
                        "tag_size_m": 0.130,
                        # Operator video is latest-only and encoded off the
                        # localisation callback. 15 Hz is responsive without
                        # spending CPU/network on frames the UI cannot use.
                        "debug_publish_rate_hz": 15.0,
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
                        "aligned_vio_odometry_topic": "/localization/fused_odom",
                        "enable_degraded_two_tag_pose": True,
                        "degraded_two_tag_inlier_corners_per_tag": 4,
                        "degraded_two_tag_max_full_rms_px": 3.0,
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
                        "output_odometry_topic": "/localization/fused_odom",
                        "map_frame": "map",
                        "base_frame": "base_link",
                        "alignment_correction_alpha": 0.25,
                        "alignment_confirm_frames": 4,
                        "alignment_candidate_max_spread_m": 0.20,
                        "alignment_candidate_max_angle_deg": 12.0,
                        "alignment_recalibration_threshold_m": 0.12,
                        "alignment_max_correction_m": 0.75,
                    }
                ],
            ),
            Node(
                package="eup_control",
                executable="thruster_allocator",
                name="thruster_allocator",
                output="screen",
            ),
            Node(
                package="eup_runtime",
                executable="safety_monitor",
                name="safety_monitor",
                output="screen",
            ),
            Node(
                package="eup_runtime",
                executable="run_logger",
                name="run_logger",
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
                        # ZED VIO owns the localisation IMU integration.
                        "publish_imu": False,
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
                        "front_camera_raw_topic": LaunchConfiguration("front_camera_raw_topic"),
                        "front_camera_tag_overlay_topic": LaunchConfiguration("front_camera_tag_overlay_topic"),
                        "front_camera_compressed_topic": LaunchConfiguration("front_camera_compressed_topic"),
                        "camera_source_lock": ParameterValue(
                            LaunchConfiguration("camera_source_lock"), value_type=bool
                        ),
                        "imu_topic": LaunchConfiguration("zed_imu_topic"),
                        "manual_thruster_span_us": ParameterValue(
                            LaunchConfiguration("manual_thruster_span_us"),
                            value_type=int,
                        ),
                        "manual_thruster_channel_offset": ParameterValue(
                            LaunchConfiguration("manual_thruster_channel_offset"),
                            value_type=int,
                        ),
                    }
                ],
            ),
        ]
    )
