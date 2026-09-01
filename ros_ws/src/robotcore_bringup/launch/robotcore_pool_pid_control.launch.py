"""Launch the fail-closed pool PID control graph without sensors or hardware."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
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
                parameters=[
                    {
                        "thruster_config_path": LaunchConfiguration(
                            "thruster_config_path"
                        ),
                        "task_catalog_path": LaunchConfiguration("task_catalog_path"),
                        "recording_config_path": LaunchConfiguration(
                            "recording_config_path"
                        ),
                        "safety_config_path": LaunchConfiguration(
                            "pool_control_config"
                        ),
                    }
                ],
            ),
        ]
    )
