"""Package metadata for the ROS-side edge hardware bridge."""

from glob import glob
from setuptools import setup

package_name = "eup_hardware"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    # The hardware config captures Phase 0 board assumptions and keeps the ROS
    # bridge separated from firmware-specific constants.
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="EUP System Team",
    maintainer_email="maintainer@example.com",
    description="Edge hardware bridge skeleton for ros2_control, serial, CAN, and Aboard packets.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "aboard_bridge_node = eup_hardware.aboard_bridge_node:main",
        ],
    },
)
