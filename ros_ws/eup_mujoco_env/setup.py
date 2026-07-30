"""Package metadata for the root-level MuJoCo backend package."""

from glob import glob
from setuptools import setup

package_name = "eup_mujoco_env"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    # Install model and asset notes with the package so a built workspace keeps
    # the simulation backend self-contained.
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/models", glob("models/*.xml")),
        (
            f"share/{package_name}/assets/bluerov2_heavy_8thruster",
            glob("assets/bluerov2_heavy_8thruster/*"),
        ),
        (
            f"share/{package_name}/assets/arm_candidates",
            glob("assets/arm_candidates/*"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="EUP System Team",
    maintainer_email="maintainer@example.com",
    description="MuJoCo backend package for EUPSystemInfraPack.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "mujoco_ros2_node = eup_mujoco_env.mujoco_ros2_node:main",
        ],
    },
)
