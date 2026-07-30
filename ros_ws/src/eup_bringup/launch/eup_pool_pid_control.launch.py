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
            Node(
                package="eup_runtime",
                executable="safety_monitor",
                name="safety_monitor",
                output="screen",
            ),
            Node(
                package="eup_runtime",
                executable="trajectory_command_node",
                name="trajectory_command",
                output="screen",
                parameters=[LaunchConfiguration("pool_control_config")],
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
                executable="tracking_monitor_node",
                name="tracking_monitor",
                output="screen",
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
                executable="run_logger",
                name="run_logger",
                output="screen",
                parameters=[
                    {
                        "pid_config_path": LaunchConfiguration("pid_config_path"),
                        "thruster_config_path": LaunchConfiguration(
                            "thruster_config_path"
                        ),
                        "scenario_config_path": LaunchConfiguration(
                            "scenario_config_path"
                        ),
                        "safety_config_path": LaunchConfiguration(
                            "pool_control_config"
                        ),
                    }
                ],
            ),
        ]
    )
