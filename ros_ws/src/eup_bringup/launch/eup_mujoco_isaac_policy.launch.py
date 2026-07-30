"""Launch MuJoCo with the exported IsaacLab WarpAUV ONNX body policy."""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Start the simulation graph using the IsaacLab ONNX body policy."""

    run_root = str(Path.cwd() / "data" / "robotcore_runs")
    isaac_policy_path = "models/policies/isaac_warpauv_traj_body_policy/policy.yaml"

    return LaunchDescription(
        [
            DeclareLaunchArgument("web_host", default_value="0.0.0.0"),
            DeclareLaunchArgument("web_port", default_value="8080"),
            DeclareLaunchArgument("enable_viewer", default_value="false"),
            DeclareLaunchArgument(
                "model_path",
                default_value="eup_mujoco_env/models/warpauv_6thruster.xml",
            ),
            DeclareLaunchArgument("trajectory_type", default_value="lissajous"),
            DeclareLaunchArgument("trajectory_amp_x", default_value="1.5"),
            DeclareLaunchArgument("trajectory_amp_y", default_value="0.75"),
            DeclareLaunchArgument("trajectory_period_s", default_value="16.0"),
            # 180 Hz physics with a 60 Hz policy action: three simulation
            # substeps run for every action update.
            DeclareLaunchArgument("physics_steps_per_tick", default_value="3"),
            DeclareLaunchArgument("control_rate_hz", default_value="60.0"),
            DeclareLaunchArgument("real_time_factor", default_value="1.0"),
            DeclareLaunchArgument("thruster_rotor_constant", default_value="0.001"),
            DeclareLaunchArgument("thruster_time_constant_s", default_value="0.05"),
            SetEnvironmentVariable(name="ROBOTCORE_RUN_ROOT", value=run_root),
            Node(
                package="eup_mujoco_env",
                executable="mujoco_ros2_node",
                name="mujoco_ros2_node",
                output="screen",
                parameters=[
                    {
                        "model_path": LaunchConfiguration("model_path"),
                        "publish_rate_hz": ParameterValue(
                            LaunchConfiguration("control_rate_hz"), value_type=float
                        ),
                        "physics_steps_per_tick": ParameterValue(
                            LaunchConfiguration("physics_steps_per_tick"),
                            value_type=int,
                        ),
                        "real_time_factor": ParameterValue(
                            LaunchConfiguration("real_time_factor"),
                            value_type=float,
                        ),
                        "enable_viewer": ParameterValue(
                            LaunchConfiguration("enable_viewer"),
                            value_type=bool,
                        ),
                        "enable_hydrodynamics": True,
                        "thruster_force_model": "isaac_warpauv",
                        "thruster_rotor_constant": ParameterValue(
                            LaunchConfiguration("thruster_rotor_constant"),
                            value_type=float,
                        ),
                        "thruster_time_constant_s": ParameterValue(
                            LaunchConfiguration("thruster_time_constant_s"),
                            value_type=float,
                        ),
                    }
                ],
            ),
            Node(
                package="eup_runtime",
                executable="trajectory_command_node",
                name="trajectory_command",
                output="screen",
                parameters=[
                    {
                        "trajectory_type": LaunchConfiguration("trajectory_type"),
                        "publish_rate_hz": ParameterValue(
                            LaunchConfiguration("control_rate_hz"), value_type=float
                        ),
                        "center_z": -1.5,
                        "amp_x": ParameterValue(
                            LaunchConfiguration("trajectory_amp_x"),
                            value_type=float,
                        ),
                        "amp_y": ParameterValue(
                            LaunchConfiguration("trajectory_amp_y"),
                            value_type=float,
                        ),
                        "period_s": ParameterValue(
                            LaunchConfiguration("trajectory_period_s"),
                            value_type=float,
                        ),
                    }
                ],
            ),
            Node(
                package="eup_runtime",
                executable="tracking_monitor_node",
                name="tracking_monitor",
                output="screen",
                parameters=[{"publish_rate_hz": 20.0}],
            ),
            Node(
                package="eup_policy",
                executable="body_policy_node",
                name="body_policy",
                output="screen",
                parameters=[
                    {
                        "policy_name": "isaac_warpauv_traj_body_policy",
                        "policy_path": isaac_policy_path,
                        "publish_rate_hz": ParameterValue(
                            LaunchConfiguration("control_rate_hz"), value_type=float
                        ),
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
                        "title": "RobotCore Isaac MuJoCo Operator",
                    }
                ],
            ),
        ]
    )
