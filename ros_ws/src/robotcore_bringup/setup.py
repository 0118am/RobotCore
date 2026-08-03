"""Package metadata for installing launch files and bringup config with colcon."""

from glob import glob
from setuptools import setup

package_name = "robotcore_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    # Launch and YAML files must be installed into share/ so ros2 launch can find
    # them after `colcon build --symlink-install`.
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="RobotCore Team",
    maintainer_email="maintainer@example.com",
    description="Launch and configuration package for RobotCore.",
    license="MIT",
)
