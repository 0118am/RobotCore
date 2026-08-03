"""Launch the real Jetson/A-board edge graph."""

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    run_root = os.environ.get(
        "ROBOTCORE_RUN_ROOT", str(Path.cwd() / "data" / "robotcore_runs")
    )

    return LaunchDescription(
        [
            # The 1a86 USB serial endpoint carries CRC protocol v2: FF FD board
            # status/ACK plus FF F8 04 external-IMU telemetry. Bind by identity
            # rather than tty numbering.
            DeclareLaunchArgument(
                "serial_port",
                default_value="/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B7A033320-if00",
            ),
            DeclareLaunchArgument("baud", default_value="115200"),
            DeclareLaunchArgument("zed_imu_topic", default_value="/zedx/zed_node/imu/data"),
            DeclareLaunchArgument("imu_raw_topic", default_value="/hardware/aboard_imu_raw"),
            DeclareLaunchArgument("external_imu_topic", default_value="/sensors/external_imu"),
            DeclareLaunchArgument(
                "external_imu_fusion_topic",
                default_value="/sensors/external_imu_specific_force",
            ),
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
            DeclareLaunchArgument("enable_fixed_rate_state_estimator", default_value="true"),
            DeclareLaunchArgument("zed_odometry_topic", default_value="/zedx/zed_node/odom"),
            DeclareLaunchArgument("zed_workspace", default_value="/home/nvidia/ros2_ws"),
            DeclareLaunchArgument(
                "zed_params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("robotcore_sensors"), "config", "zedx_minimal_open.yaml"]
                ),
            ),
            DeclareLaunchArgument("zed_serial_number", default_value="50649148"),
            DeclareLaunchArgument("zed_camera_id", default_value="-1"),
            DeclareLaunchArgument(
                "external_imu_config",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("robotcore_sensors"), "config", "external_imu.yaml"]
                ),
            ),
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
            # Production systemd units set this false and run control_interface as the
            # separate, least-privileged control-interface.service.
            DeclareLaunchArgument("enable_web_ui", default_value="true"),
            # Constrain every source (web, policy, and gamepad) to
            # 1400–1600 us around a 1500 us neutral command.
            DeclareLaunchArgument("manual_thruster_span_us", default_value="100"),
            DeclareLaunchArgument("thruster_command_timeout_ms", default_value="150"),
            DeclareLaunchArgument(
                "pool_control_config",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("robotcore_control"), "config", "real_pool_safety.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "pid_config_path",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("robotcore_control"), "config", "real_pool_pid.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "thruster_config_path",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("robotcore_control"), "config", "real_pool_thrusters.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "scenario_config_path",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("robotcore_runtime"), "config", "tracking_scenarios.yaml"]
                ),
            ),
            SetEnvironmentVariable(name="ROBOTCORE_RUN_ROOT", value=run_root),
            # Static transforms use the standard C++ tf2 publisher. No Python
            # process remains in the sensor-to-BodyState data path.
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_to_zed_link",
                output="screen",
                arguments=[
                    "--x", "0.236", "--y", "0.027", "--z", "0.016",
                    "--frame-id", "base_link", "--child-frame-id", "zedx_camera_link",
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_to_aboard_imu",
                output="screen",
                arguments=[
                    "--x", "0.018", "--z", "0.076",
                    "--frame-id", "base_link", "--child-frame-id", "aboard_imu_link",
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_to_front_camera_optical",
                output="screen",
                arguments=[
                    "--x", "0.236", "--y", "0.027", "--z", "0.016",
                    "--roll", "-1.5707963267948966", "--yaw", "-1.5707963267948966",
                    "--frame-id", "base_link", "--child-frame-id", "front_camera_optical_frame",
                ],
            ),
            ExecuteProcess(
                cmd=[
                    "/usr/bin/bash",
                    "-c",
                    [
                        "source ", LaunchConfiguration("zed_workspace"),
                        "/install/setup.bash && exec ros2 launch zed_wrapper zed_camera.launch.py ",
                        "camera_model:=zedxm camera_name:=zedx serial_number:=",
                        LaunchConfiguration("zed_serial_number"),
                        " camera_id:=", LaunchConfiguration("zed_camera_id"),
                        " publish_urdf:=true publish_tf:=false publish_map_tf:=false ",
                        "publish_imu_tf:=false enable_ipc:=false node_log_type:=screen ",
                        "ros_params_override_path:=", LaunchConfiguration("zed_params_file"),
                    ],
                ],
                output="screen",
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
                parameters=[{"thread_num": 2}],
                condition=IfCondition(LaunchConfiguration("enable_apriltag_localization")),
                composable_node_descriptions=[
                    # ZED publishes BGR8 NITROS images, while the CUDA
                    # AprilTag node requests RGB8. Keep the conversion on GPU
                    # so format negotiation succeeds without a CPU/DDS image
                    # round trip.
                    ComposableNode(
                        package="isaac_ros_image_proc",
                        plugin="nvidia::isaac_ros::image_proc::ImageFormatConverterNode",
                        name="apriltag_bgr_to_rgb",
                        parameters=[{
                            "encoding_desired": "rgb8",
                            # Explicitly pin both ends. Without the input
                            # constraint the compatible subscriber falls back
                            # to RGB8 and cannot negotiate with ZED's BGR8
                            # NITROS publisher.
                            "image_raw_nitros_format": "nitros_image_bgr8",
                            "image_nitros_format": "nitros_image_rgb8",
                            "image_width": 960,
                            "image_height": 600,
                        }],
                        remappings=[
                            ("image_raw", LaunchConfiguration("front_camera_raw_topic")),
                            ("image", "/localization/apriltag/image_rgb"),
                        ],
                    ),
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
                            # The CUDA converter resolves ZED BGR8 to the RGB8
                            # format required by AprilTag entirely in NITROS.
                            ("image", "/localization/apriltag/image_rgb"),
                            ("camera_info", LaunchConfiguration("front_camera_info_topic")),
                            (
                                "tag_detections",
                                LaunchConfiguration("apriltag_detections_topic"),
                            ),
                            # Never let the detector's one-size tag poses join
                            # the production TF tree.
                            ("tf", "/localization/apriltag/raw_tf"),
                        ],
                    ),
                    ComposableNode(
                        package="robotcore_sensors",
                        plugin="robotcore_sensors::AprilTagMapLocalizerComponent",
                        name="apriltag_localization",
                        parameters=[{
                            "camera_info_topic": LaunchConfiguration("front_camera_info_topic"),
                            "detections_topic": LaunchConfiguration("apriltag_detections_topic"),
                            "tag_map_file": LaunchConfiguration("apriltag_tag_map_file"),
                            "detected_count_topic": LaunchConfiguration("apriltag_detected_count_topic"),
                            "pose_status_topic": "/localization/apriltag/pose_status",
                            "base_frame": "base_link",
                            "map_frame": "map",
                            "enforce_cuboid_pool_geometry": True,
                            "pool_length_m": 5.42,
                            "pool_width_m": 3.73,
                            "pool_surface_tolerance_m": 0.02,
                            "pool_orientation_tolerance_deg": 2.0,
                            "minimum_pose_tag_count": 3,
                            "minimum_inlier_corners_per_tag": 3,
                            "max_reprojection_rms_px": 3.0,
                            "max_translation_jump_m": 0.05,
                            "multi_tag_position_stddev_m": 0.05,
                            "base_to_camera_translation_m": [0.236, 0.027, 0.016],
                        }],
                        extra_arguments=[{"use_intra_process_comms": True}],
                    ),
                ],
            ),
            ComposableNodeContainer(
                package="rclcpp_components",
                executable="component_container_mt",
                name="localization_estimator_container",
                namespace="",
                output="screen",
                parameters=[{"thread_num": 3}],
                condition=IfCondition(LaunchConfiguration("enable_fixed_rate_state_estimator")),
                composable_node_descriptions=[
                    ComposableNode(
                        package="robotcore_sensors",
                        plugin="robotcore_sensors::ImuConditionerComponent",
                        name="imu_conditioning",
                        parameters=[LaunchConfiguration("external_imu_config"), {
                            "input_topic": LaunchConfiguration("imu_raw_topic"),
                            "output_topic": LaunchConfiguration("external_imu_topic"),
                            "fusion_output_topic": LaunchConfiguration(
                                "external_imu_fusion_topic"
                            ),
                            "calibration_sample_count": 250,
                            "calibration_timeout_s": 10.0,
                        }],
                        extra_arguments=[{"use_intra_process_comms": True}],
                    ),
                    ComposableNode(
                        package="robotcore_sensors",
                        plugin="robotcore_sensors::ZedOdometryAdapterComponent",
                        name="zed_odometry_adapter",
                        parameters=[{
                            "input_topic": LaunchConfiguration("zed_odometry_topic"),
                            "output_topic": "/localization/zed_odom",
                            "base_frame": "base_link",
                        }],
                        extra_arguments=[{"use_intra_process_comms": True}],
                    ),
                    ComposableNode(
                        package="robotcore_sensors",
                        plugin="robotcore_sensors::FixedLagEskfComponent",
                        name="fixed_lag_eskf",
                        parameters=[{
                            "imu_topic": LaunchConfiguration("external_imu_fusion_topic"),
                            "vio_topic": "/localization/zed_odom",
                            "tag_topic": "/localization/apriltag_pose",
                            "output_rate_hz": 60.0,
                            "history_duration_s": 3.0,
                            "alignment_correction_alpha": 0.25,
                            "alignment_max_correction_m": 0.75,
                            "alignment_max_correction_angle_deg": 20.0,
                        }],
                        extra_arguments=[{"use_intra_process_comms": True}],
                    ),
                ],
            ),
            Node(
                package="robotcore_runtime",
                executable="trajectory_command_node",
                name="trajectory_command",
                output="screen",
                parameters=[LaunchConfiguration("pool_control_config")],
            ),
            Node(
                package="robotcore_runtime",
                executable="tracking_monitor_node",
                name="tracking_monitor",
                output="screen",
                parameters=[{"publish_rate_hz": 20.0}],
            ),
            Node(
                package="robotcore_control",
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
                package="robotcore_control",
                executable="command_authority",
                name="command_authority",
                output="screen",
                parameters=[LaunchConfiguration("pool_control_config")],
            ),
            Node(
                package="robotcore_runtime",
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
                package="robotcore_runtime",
                executable="safety_monitor",
                name="safety_monitor",
                output="screen",
            ),
            Node(
                package="robotcore_runtime",
                executable="blackboard",
                name="blackboard",
                output="screen",
            ),
            Node(
                package="robotcore_hardware",
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
                        "command_timeout_ms": ParameterValue(
                            LaunchConfiguration("thruster_command_timeout_ms"),
                            value_type=int,
                        ),
                        # C++ bridge accepts only versioned, CRC-valid frame-4
                        # samples with authoritative MCU acquisition stamps.
                        "imu_topic": LaunchConfiguration("imu_raw_topic"),
                        "imu_frame_id": "aboard_imu_link",
                    }
                ],
            ),
            Node(
                package="control_interface",
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
                        "manual_thruster_topic": "/control/candidates/manual",
                    }
                ],
            ),
        ]
    )
