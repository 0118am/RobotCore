"""Package metadata for real sensor bridge nodes."""

from glob import glob
from setuptools import setup

package_name = "eup_sensors"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", glob("config/*.yaml") + glob("config/*.json")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="EUP System Team",
    maintainer_email="maintainer@example.com",
    description="Real sensor bridge and frame transform nodes.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "sensor_fusion_node = eup_sensors.sensor_fusion_node:main",
            "vehicle_frames_node = eup_sensors.vehicle_frames_node:main",
            "apriltag_localization_node = eup_sensors.apriltag_localization_node:main",
            "zed_odometry_adapter_node = eup_sensors.zed_odometry_adapter_node:main",
            "tag_vio_alignment_node = eup_sensors.tag_vio_alignment_node:main",
        ],
    },
)
