"""Package metadata for policy runtime nodes and runner interfaces."""

from glob import glob
from setuptools import setup

package_name = "robotcore_policy"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name, f"{package_name}.runners"],
    # policy_runtime.yaml records the default manifest paths used by bringup.
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="RobotCore Team",
    maintainer_email="maintainer@example.com",
    description="Policy runtime, runners, observation builders, and action decoders.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "arm_policy_node = robotcore_policy.arm_policy_node:main",
            "body_policy_node = robotcore_policy.body_policy_node:main",
        ],
    },
)
