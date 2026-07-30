"""Package metadata for control nodes and their YAML configuration."""

from glob import glob
from setuptools import setup

package_name = "eup_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    # Install control.yaml so allocator/PWM settings can later be loaded by
    # launch files or parameter overrides instead of being hardcoded.
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="EUP System Team",
    maintainer_email="maintainer@example.com",
    description="Thruster allocation, arm command routing, and safety filters.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "arm_controller = eup_control.arm_controller:main",
            "command_authority = eup_control.command_authority_node:main",
            "pid_controller = eup_control.six_dof_pid_node:main",
            "rl_action_adapter = eup_control.rl_action_adapter:main",
            "thruster_allocator = eup_control.thruster_allocator:main",
        ],
    },
)
