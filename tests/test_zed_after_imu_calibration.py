"""ROS-environment tests for the direct ZED launcher."""

import sys
from pathlib import Path

import pytest


CORE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(CORE_ROOT / "ros_ws/src/eup_runtime"))
rclpy = pytest.importorskip("rclpy")

from eup_runtime.zed_camera_launcher import zed_launch_command


def test_zed_command_sources_only_the_configured_zed_workspace():
    command = zed_launch_command(
        workspace="/tmp/zed workspace",
        params_file="/tmp/zed params.yaml",
        serial_number="50649148",
        camera_id="-1",
    )

    assert command[:2] == ["/usr/bin/bash", "-c"]
    script = command[2]
    assert "source '/tmp/zed workspace/install/setup.bash'" in script
    assert "ros2 launch zed_wrapper zed_camera.launch.py" in script
    assert "serial_number:=50649148" in script
    assert "camera_id:=-1" in script
    assert "publish_tf:=false" in script
    assert "'ros_params_override_path:=/tmp/zed params.yaml'" in script
