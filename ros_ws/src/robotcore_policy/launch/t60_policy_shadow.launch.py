"""Launch model_499 for observation/inference logging without actuation."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("robotcore_policy"))
    default_manifest = (
        package_share / "models" / "t60_precision_v7_model_499" / "policy.yaml"
    )
    manifest_argument = DeclareLaunchArgument(
        "policy_manifest", default_value=str(default_manifest)
    )
    policy_node = Node(
        package="robotcore_policy",
        executable="body_policy_node",
        name="t60_policy_shadow",
        output="screen",
        parameters=[
            {
                "policy_name": "t60_precision_v7_model_499",
                "policy_path": LaunchConfiguration("policy_manifest"),
                # Explicit here as a second guard against accidental rate drift.
                "publish_rate_hz": 25.0,
            }
        ],
    )
    return LaunchDescription([manifest_argument, policy_node])
