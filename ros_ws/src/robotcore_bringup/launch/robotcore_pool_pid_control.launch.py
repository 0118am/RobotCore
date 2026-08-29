"""Launch the fail-closed pool PID control graph without sensors or hardware."""

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    task_config_dir = (
        Path(
            os.environ.get(
                "CONTROL_INTERFACE_WORKSPACE", "/home/nvidia/ControlInterface"
            )
        )
        / "control_interface"
        / "config"
        / "tasks"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "pool_control_config",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("robotcore_control"), "config", "real_pool_safety.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "pid_config_path",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("robotcore_control"), "config", "pid", "default.json"]
                ),
            ),
            DeclareLaunchArgument(
                "thruster_config_path",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("robotcore_control"), "config", "real_pool_thrusters.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "task_config_dir",
                default_value=str(task_config_dir),
            ),
            DeclareLaunchArgument(
                "record_topics_path",
                default_value=str(task_config_dir / "record_topics.json"),
            ),
            Node(
                package="robotcore_runtime",
                executable="safety_monitor",
                name="safety_monitor",
                output="screen",
            ),
            Node(
                package="robotcore_runtime",
                executable="trajectory_command_node",
                name="trajectory_command",
                output="screen",
                parameters=[LaunchConfiguration("pool_control_config")],
            ),
            Node(
                package="robotcore_control",
                executable="pid_controller",
                name="pid_controller",
                output="screen",
                parameters=[
                    LaunchConfiguration("pool_control_config"),
                    {
                        "pid_config_path": LaunchConfiguration("pid_config_path"),
                        "thruster_config_path": LaunchConfiguration(
                            "thruster_config_path"
                        ),
                    }
                ],
            ),
            Node(
                package="robotcore_control_cpp",
                executable="command_authority",
                name="command_authority",
                output="screen",
                parameters=[LaunchConfiguration("pool_control_config")],
            ),
            Node(
                package="robotcore_runtime",
                executable="tracking_monitor_node",
                name="tracking_monitor",
                output="screen",
            ),
            Node(
                package="robotcore_runtime",
                executable="tracking_experiment_node",
                name="tracking_experiment",
                output="screen",
                parameters=[
                    {
                        "task_config_dir": LaunchConfiguration(
                            "task_config_dir"
                        )
                    }
                ],
            ),
            Node(
                package="robotcore_runtime",
                executable="run_logger",
                name="run_logger",
                output="screen",
                parameters=[
                    {
                        "pid_config_path": LaunchConfiguration("pid_config_path"),
                        "thruster_config_path": LaunchConfiguration(
                            "thruster_config_path"
                        ),
                        "task_config_dir": LaunchConfiguration("task_config_dir"),
                        "record_topics_path": LaunchConfiguration("record_topics_path"),
                        "safety_config_path": LaunchConfiguration(
                            "pool_control_config"
                        ),
                    }
                ],
            ),
        ]
    )
