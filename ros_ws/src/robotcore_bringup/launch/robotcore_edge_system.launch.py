"""Launch the real Jetson/Aquaboard edge graph."""

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    SetEnvironmentVariable,
    TimerAction,
)
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
            DeclareLaunchArgument(
                "imu_yaw_offset_deg",
                # Optional startup fallback. The bridge replaces it from live
                # BodyState and raw IMU samples after every relocalization.
                default_value="0.0",
            ),
            DeclareLaunchArgument("front_camera_raw_topic", default_value="/zedx/zed_node/rgb/color/rect/image"),
            DeclareLaunchArgument(
                "front_camera_info_topic", default_value="/zedx/zed_node/rgb/color/rect/camera_info"
            ),
            DeclareLaunchArgument("enable_apriltag_localization", default_value="true"),
            # The validation-vehicle thruster model and pool envelope are
            # confirmed. Individual authority, localisation and target gates
            # remain fail-closed even though the tracking processes are live.
            DeclareLaunchArgument("enable_pool_tracking", default_value="true"),
            DeclareLaunchArgument("enable_rl_policy_runtime", default_value="true"),
            # The policy is trained in the physical T1-T8 command convention.
            # Arming, localisation, target freshness and safety gates remain mandatory.
            DeclareLaunchArgument("allow_rl_hardware", default_value="true"),
            DeclareLaunchArgument(
                "rl_policy_model",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("robotcore_policy"),
                        "models",
                        "t60_precision_v17_model_400",
                        "policy.onnx",
                    ]
                ),
            ),
            DeclareLaunchArgument(
                "apriltag_detections_topic",
                default_value="/localization/apriltag/detections",
            ),
            # ZED VIO supplies continuous local pose/twist and AprilTag supplies
            # absolute map corrections. External IMU bypasses this EKF and goes
            # directly to the control and operator paths.
            DeclareLaunchArgument("enable_fixed_rate_state_estimator", default_value="true"),
            DeclareLaunchArgument("zed_workspace", default_value="/home/nvidia/ros2_ws"),
            DeclareLaunchArgument(
                "zed_params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("robotcore_sensors"), "config", "zedx_minimal_open.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "localization_config",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("robotcore_sensors"), "config", "localization.yaml"]
                ),
            ),
            DeclareLaunchArgument("zed_serial_number", default_value="50649148"),
            DeclareLaunchArgument("zed_camera_id", default_value="-1"),
            DeclareLaunchArgument("apriltag_max_tags", default_value="24"),
            DeclareLaunchArgument(
                "apriltag_tag_map_file",
                # The host manager and the separate web UI use this one
                # deployed map.  Keep it out of the installed package so a
                # web edit and a localisation reload always refer to exactly
                # the same file.
                default_value="/etc/robotcore/apriltag_map.json",
            ),
            DeclareLaunchArgument("thruster_command_timeout_ms", default_value="150"),
            DeclareLaunchArgument(
                "pool_control_config",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("robotcore_control"), "config", "real_pool_safety.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "thruster_config_path",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("robotcore_control"), "config", "real_pool_thrusters.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "task_catalog_path",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("robotcore_runtime"), "config", "tracking_tasks.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "recording_config_path",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("robotcore_runtime"), "config", "recording.yaml"]
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
                prefix="/usr/bin/taskset --cpu-list 7 /usr/bin/nice -n 19",
                arguments=[
                    "--x", "0.236", "--y", "0.027", "--z", "0.016",
                    "--frame-id", "base_link", "--child-frame-id", "zedx_camera_link",
                ],
            ),
            ExecuteProcess(
                cmd=[
                    "/usr/bin/taskset",
                    "--cpu-list",
                    "4,5",
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
            # map contains both 0.4 m and 0.2 m Tags. RobotCore's own
            # apriltag_localization component consumes ID/corners and performs
            # one joint, per-Tag-size PnP.
            # Let the ZED publisher finish camera initialization before the
            # downstream GPU graph is created.  The installed AprilTag 3.2.5
            # node accepts RGB8 while ZED supplies BGR8, so an accelerated
            # format converter performs the channel swap without a CPU image.
            TimerAction(
                period=12.0,
                actions=[
                    ComposableNodeContainer(
                        package="rclcpp_components",
                        executable="component_container_mt",
                        name="apriltag_cuda_container",
                        namespace="",
                        output="screen",
                        prefix="/usr/bin/taskset --cpu-list 6,7",
                        parameters=[{"thread_num": 2}],
                        condition=IfCondition(
                            LaunchConfiguration("enable_apriltag_localization")
                        ),
                        composable_node_descriptions=[
                            ComposableNode(
                                package="isaac_ros_image_proc",
                                plugin=(
                                    "nvidia::isaac_ros::image_proc::"
                                    "ImageFormatConverterNode"
                                ),
                                name="apriltag_rgb_converter",
                                parameters=[
                                    {
                                        "encoding_desired": "rgb8",
                                        "image_width": 960,
                                        "image_height": 600,
                                    }
                                ],
                                remappings=[
                                    (
                                        "image_raw",
                                        LaunchConfiguration("front_camera_raw_topic"),
                                    ),
                                    (
                                        "image",
                                        "/localization/apriltag/rgb_image",
                                    ),
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
                                    # ZED's 24-bit NITROS publisher already provides
                                    # GPU-resident BGR8. The converter above supplies
                                    # the RGB8 format required by AprilTag 3.2.5.
                                    (
                                        "image",
                                        "/localization/apriltag/rgb_image",
                                    ),
                                    (
                                        "camera_info",
                                        LaunchConfiguration("front_camera_info_topic"),
                                    ),
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
                                parameters=[
                                    LaunchConfiguration("localization_config"),
                                    {
                                        "camera_info_topic": LaunchConfiguration(
                                            "front_camera_info_topic"
                                        ),
                                        "detections_topic": LaunchConfiguration(
                                            "apriltag_detections_topic"
                                        ),
                                        "tag_map_file": LaunchConfiguration(
                                            "apriltag_tag_map_file"
                                        ),
                                    },
                                ],
                                extra_arguments=[{"use_intra_process_comms": True}],
                            ),
                        ],
                    ),
                ],
            ),
            ComposableNodeContainer(
                package="rclcpp_components",
                executable="component_container",
                name="localization_estimator_container",
                namespace="",
                output="screen",
                prefix="/usr/bin/taskset --cpu-list 3",
                condition=IfCondition(LaunchConfiguration("enable_fixed_rate_state_estimator")),
                composable_node_descriptions=[
                    ComposableNode(
                        package="robotcore_sensors",
                        plugin="robotcore_sensors::VioTagFusionComponent",
                        name="ekf",
                        parameters=[
                            LaunchConfiguration("localization_config"),
                            {"executor_realtime_priority": 55},
                        ],
                        extra_arguments=[{"use_intra_process_comms": True}],
                    ),
                ],
            ),
            Node(
                package="robotcore_runtime",
                executable="trajectory_command_node",
                name="trajectory_command",
                output="screen",
                prefix="/usr/bin/taskset --cpu-list 7 /usr/bin/nice -n 10",
                condition=IfCondition(LaunchConfiguration("enable_pool_tracking")),
                parameters=[
                    LaunchConfiguration("pool_control_config"),
                    {"imu_topic": "/sensors/external_imu"},
                ],
            ),
            Node(
                package="robotcore_runtime",
                executable="tracking_monitor_node",
                name="tracking_monitor",
                output="screen",
                prefix="/usr/bin/taskset --cpu-list 7 /usr/bin/nice -n 19",
                condition=IfCondition(LaunchConfiguration("enable_pool_tracking")),
                parameters=[{"publish_rate_hz": 20.0}],
            ),
            Node(
                package="robotcore_control",
                executable="pid_controller",
                name="pid_controller",
                output="screen",
                # CPU 3 is owned by the SCHED_FIFO/55 state estimator.  A
                # normal-priority PID timer on that core can miss the 100 ms
                # command-authority freshness window while the estimator is
                # busy.  CPU 6 only carries lower-nice perception work.
                prefix="/usr/bin/taskset --cpu-list 6",
                condition=IfCondition(LaunchConfiguration("enable_pool_tracking")),
                parameters=[
                    LaunchConfiguration("pool_control_config"),
                    {
                        "control_rate_hz": 50.0,
                        "imu_topic": "/sensors/external_imu",
                    }
                ],
            ),
            Node(
                package="robotcore_policy_cpp",
                executable="t60_policy",
                name="t60_policy",
                output="screen",
                # Keep policy inference off CPU 3, where the higher-priority
                # VIO/Tag EKF executor can otherwise starve its 25 Hz timer.
                prefix="/usr/bin/taskset --cpu-list 7",
                condition=IfCondition(
                    LaunchConfiguration("enable_rl_policy_runtime")
                ),
                parameters=[
                    {
                        "policy_name": "t60_precision_v17_model_400",
                        "model_path": LaunchConfiguration("rl_policy_model"),
                        "control_rate_hz": 25.0,
                        "max_input_age_s": 0.25,
                        "executor_realtime_priority": 50,
                        # Small, bounded physical rate feedback after the RL
                        # actor. It adds damping without adding filter delay.
                        "rate_damping_enabled": True,
                        "roll_rate_damping_gain_action_per_rps": 0.10,
                        "pitch_rate_damping_gain_action_per_rps": 0.10,
                        "rate_damping_action_limit": 0.08,
                    }
                ],
            ),
            Node(
                package="robotcore_control_cpp",
                executable="command_authority",
                name="command_authority",
                output="screen",
                prefix="/usr/bin/taskset --cpu-list 2",
                parameters=[
                    LaunchConfiguration("pool_control_config"),
                    {
                        "allow_rl_hardware": ParameterValue(
                            LaunchConfiguration("allow_rl_hardware"),
                            value_type=bool,
                        ),
                        "executor_realtime_priority": 65,
                    },
                ],
            ),
            Node(
                package="robotcore_runtime",
                executable="tracking_experiment_node",
                name="tracking_experiment",
                output="screen",
                prefix="/usr/bin/taskset --cpu-list 7 /usr/bin/nice -n 19",
                condition=IfCondition(LaunchConfiguration("enable_pool_tracking")),
                parameters=[
                    {
                        "task_catalog_path": LaunchConfiguration(
                            "task_catalog_path"
                        )
                    }
                ],
            ),
            Node(
                package="robotcore_runtime",
                executable="run_logger",
                name="run_logger",
                output="screen",
                prefix="/usr/bin/taskset --cpu-list 7 /usr/bin/nice -n 19",
                condition=IfCondition(LaunchConfiguration("enable_pool_tracking")),
                parameters=[
                    {
                        "run_root": run_root,
                        "thruster_config_path": LaunchConfiguration("thruster_config_path"),
                        "task_catalog_path": LaunchConfiguration("task_catalog_path"),
                        "recording_config_path": LaunchConfiguration(
                            "recording_config_path"
                        ),
                        "safety_config_path": LaunchConfiguration("pool_control_config"),
                    }
                ],
            ),
            Node(
                package="robotcore_runtime",
                executable="safety_monitor",
                name="safety_monitor",
                output="screen",
                prefix="/usr/bin/taskset --cpu-list 2",
            ),
            Node(
                package="robotcore_hardware",
                executable="aboard_bridge_node",
                name="aboard_bridge",
                output="screen",
                prefix="/usr/bin/taskset --cpu-list 2",
                parameters=[
                    {
                        "serial_port": LaunchConfiguration("serial_port"),
                        "baud": ParameterValue(LaunchConfiguration("baud"), value_type=int),
                        "imu_yaw_offset_deg": ParameterValue(
                            LaunchConfiguration("imu_yaw_offset_deg"), value_type=float
                        ),
                        "command_timeout_ms": ParameterValue(
                            LaunchConfiguration("thruster_command_timeout_ms"),
                            value_type=int,
                        ),
                        "executor_realtime_priority": 65,
                        "serial_realtime_priority": 70,
                        "imu_realtime_priority": 60,
                    }
                ],
            ),
        ]
    )
