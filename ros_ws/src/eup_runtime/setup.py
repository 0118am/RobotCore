"""Package metadata for runtime, safety, task, and logging nodes."""

from glob import glob
from setuptools import setup

package_name = "eup_runtime"

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
    maintainer="EUP System Team",
    maintainer_email="maintainer@example.com",
    description="Runtime, safety, blackboard, task, and logging orchestration.",
    license="MIT",
    # Console scripts are the ROS node executables referenced by bringup launch
    # files. Keeping them explicit helps review the runtime graph.
    entry_points={
        "console_scripts": [
            "blackboard = eup_runtime.blackboard:main",
            "run_logger = eup_runtime.run_logger:main",
            "safety_monitor = eup_runtime.safety_monitor:main",
            "task_manager = eup_runtime.task_manager:main",
            "trajectory_command_node = eup_runtime.trajectory_command_node:main",
            "tracking_monitor_node = eup_runtime.tracking_monitor_node:main",
            "tracking_experiment_node = eup_runtime.tracking_experiment_node:main",
            "zed_camera_launcher = eup_runtime.zed_camera_launcher:main",
        ],
    },
)
