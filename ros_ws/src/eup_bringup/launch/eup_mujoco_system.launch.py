"""Launch the Phase 2+ graph using MuJoCo as the hardware backend."""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Use the same run-root convention as the edge graph so replay artifacts are
    # comparable across simulated and hardware runs.
    run_root = str(Path.cwd() / "data" / "robotcore_runs")

    return LaunchDescription(
        [
            DeclareLaunchArgument("web_host", default_value="0.0.0.0"),
            DeclareLaunchArgument("web_port", default_value="8080"),
            DeclareLaunchArgument("enable_viewer", default_value="false"),
            SetEnvironmentVariable(name="ROBOTCORE_RUN_ROOT", value=run_root),
            Node(
                package="eup_mujoco_env",
                executable="mujoco_ros2_node",
                name="mujoco_ros2_node",
                output="screen",
                # The bridge accepts a relative model path so this launch file
                # remains portable inside the workspace during Phase 2.
                parameters=[
                    {
                        "model_path": "eup_mujoco_env/models/bluerov2_heavy_generic.xml",
                        "publish_rate_hz": 50.0,
                        "enable_viewer": ParameterValue(
                            LaunchConfiguration("enable_viewer"),
                            value_type=bool,
                        ),
                    }
                ],
            ),
            Node(
                package="eup_policy",
                executable="body_policy_node",
                name="body_policy",
                output="screen",
                parameters=[
                    {
                        "policy_name": "dummy_body_policy",
                        "policy_path": "models/policies/dummy_body_policy/policy.yaml",
                        "publish_rate_hz": 10.0,
                    }
                ],
            ),
            Node(
                package="eup_policy",
                executable="arm_policy_node",
                name="arm_policy",
                output="screen",
                parameters=[
                    {
                        "policy_name": "dummy_arm_policy",
                        "policy_path": "models/policies/dummy_arm_policy/policy.yaml",
                        "publish_rate_hz": 10.0,
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
                package="eup_control",
                executable="arm_controller",
                name="arm_controller",
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
                package="eup_ui",
                executable="web_operator_server",
                name="web_operator",
                output="screen",
                parameters=[
                    {
                        "web_host": LaunchConfiguration("web_host"),
                        "web_port": LaunchConfiguration("web_port"),
                        "title": "RobotCore MuJoCo Operator",
                    }
                ],
            ),
        ]
    )
