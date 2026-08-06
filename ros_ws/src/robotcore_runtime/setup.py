"""Package metadata for runtime, safety, task, and logging nodes."""

from glob import glob
from setuptools import setup

package_name = "robotcore_runtime"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="RobotCore Team",
    maintainer_email="maintainer@example.com",
    description="Runtime, safety, blackboard, task, and logging orchestration.",
    license="MIT",
    # Console scripts are the ROS node executables referenced by bringup launch
    # files. Keeping them explicit helps review the runtime graph.
    entry_points={
        "console_scripts": [
            "blackboard = robotcore_runtime.blackboard:main",
            "run_logger = robotcore_runtime.run_logger:main",
            "safety_monitor = robotcore_runtime.safety_monitor:main",
            "task_manager = robotcore_runtime.task_manager:main",
            "trajectory_command_node = robotcore_runtime.trajectory_command_node:main",
            "tracking_monitor_node = robotcore_runtime.tracking_monitor_node:main",
            "tracking_experiment_node = robotcore_runtime.tracking_experiment_node:main",
        ],
    },
)
