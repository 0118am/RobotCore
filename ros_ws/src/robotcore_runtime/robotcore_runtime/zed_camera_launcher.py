"""Launch the ZED driver immediately as part of the edge graph."""

from __future__ import annotations

import os
from pathlib import Path
import shlex
import signal
import subprocess

import rclpy
from rclpy.node import Node


def zed_launch_command(
    *,
    workspace: str,
    params_file: str,
    serial_number: str,
    camera_id: str,
) -> list[str]:
    """Build the fixed ZED launch command without accepting browser input."""

    setup_file = Path(workspace).expanduser() / "install" / "setup.bash"
    launch_arguments = [
        "camera_model:=zedxm",
        "camera_name:=zedx",
        f"serial_number:={serial_number}",
        f"camera_id:={camera_id}",
        # ZED owns its internal IMU fusion for VIO. Its static camera frame
        # tree is needed to rotate VIO twist into base_link; dynamic odom/map
        # TF remains disabled because Tag/VIO alignment owns the map pose.
        "publish_urdf:=true",
        "publish_tf:=false",
        "publish_map_tf:=false",
        "publish_imu_tf:=false",
        # NITROS owns the GPU image path. ZED's launch file intentionally
        # treats its separate TypeAdapter IPC mode as mutually exclusive.
        "enable_ipc:=false",
        "node_log_type:=screen",
        f"ros_params_override_path:={params_file}",
    ]
    command = ["ros2", "launch", "zed_wrapper", "zed_camera.launch.py", *launch_arguments]
    script = f"source {shlex.quote(str(setup_file))} && exec {shlex.join(command)}"
    return ["/usr/bin/bash", "-c", script]


class ZedCameraLauncher(Node):
    """Own the ZED child process for the lifetime of the edge launch."""

    def __init__(self):
        super().__init__("zed_camera_launcher")
        self.declare_parameter("zed_workspace", "/home/nvidia/ros2_ws")
        self.declare_parameter(
            "zed_params_file",
            "/home/nvidia/RobotCore/ros_ws/src/robotcore_sensors/config/zedx_minimal_open.yaml",
        )
        self.declare_parameter("zed_serial_number", "50649148")
        self.declare_parameter("zed_camera_id", "-1")

        self._process: subprocess.Popen[bytes] | None = None
        self._exit_reported = False
        workspace = str(self.get_parameter("zed_workspace").value)
        setup_file = Path(workspace).expanduser() / "install" / "setup.bash"
        if not setup_file.is_file():
            self.get_logger().error(f"ZED workspace setup file is unavailable: {setup_file}")
        else:
            command = zed_launch_command(
                workspace=workspace,
                params_file=str(self.get_parameter("zed_params_file").value),
                serial_number=str(self.get_parameter("zed_serial_number").value),
                camera_id=str(self.get_parameter("zed_camera_id").value),
            )
            try:
                self._process = subprocess.Popen(command, start_new_session=True)
            except OSError as exc:
                self.get_logger().error(f"Unable to launch ZED camera: {exc}")
            else:
                self.get_logger().info("Starting ZED camera without an A-board IMU gate")
        self.create_timer(1.0, self._report_camera_exit)

    def _report_camera_exit(self) -> None:
        if self._process is None or self._exit_reported:
            return
        exit_code = self._process.poll()
        if exit_code is not None:
            self._exit_reported = True
            self.get_logger().error(f"ZED camera launch exited with code {exit_code}")

    def destroy_node(self):
        process = self._process
        if process is not None and process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGINT)
                process.wait(timeout=8.0)
            except (OSError, subprocess.TimeoutExpired):
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except OSError:
                    pass
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ZedCameraLauncher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
