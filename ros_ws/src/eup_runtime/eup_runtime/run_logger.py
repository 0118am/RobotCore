"""Run logger node.

This node creates the acceptance-test run folder and writes lightweight JSONL
records for events and command/status streams. rosbag2 recording can be added
around the same run directory in a later phase.
"""

import json
import os
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from eup_interfaces.msg import (
    ArmCommand,
    PolicyStatus,
    SafetyEvent,
    ThrusterCommand,
    TrackingStatus,
    TrajectoryTarget,
)


class RunLogger(Node):
    """Creates the run folder and writes lightweight JSONL event records."""

    def __init__(self):
        super().__init__("run_logger")
        # ROBOTCORE_RUN_ROOT lets launch files keep log output in the workspace while
        # still allowing deployment scripts to redirect logs to mounted storage.
        self.declare_parameter("run_root", os.environ.get("ROBOTCORE_RUN_ROOT", "data/robotcore_runs"))
        root = Path(self.get_parameter("run_root").value)
        self.run_dir = self.create_run_dir(root)
        self.event_log_path = self.run_dir / "event_log.jsonl"
        # Policy nodes subscribe to this topic so all policy_io files land in
        # the same run folder without sharing process-local state.
        self.run_dir_pub = self.create_publisher(String, "/runtime/run_dir", 10)

        self.create_subscription(SafetyEvent, "/safety/events", self.on_safety_event, 20)
        self.create_subscription(
            PolicyStatus, "/policy/body/status", self.on_policy_status, 20
        )
        self.create_subscription(
            PolicyStatus, "/policy/arm/status", self.on_policy_status, 20
        )
        self.create_subscription(
            ThrusterCommand, "/control/thruster_cmd", self.on_thruster_cmd, 20
        )
        self.create_subscription(ArmCommand, "/control/arm_cmd", self.on_arm_cmd, 20)
        self.create_subscription(
            TrajectoryTarget, "/runtime/trajectory_target", self.on_trajectory_target, 20
        )
        self.create_subscription(
            TrackingStatus, "/runtime/tracking_status", self.on_tracking_status, 20
        )

        self.write_event(
            "run_started",
            {
                "run_dir": str(self.run_dir),
                "rosbag2_dir": str(self.run_dir / "rosbag2"),
            },
        )
        self.get_logger().info(f"Run folder: {self.run_dir}")
        self.timer = self.create_timer(1.0, self.publish_run_dir)

    def create_run_dir(self, root):
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        run_dir = root / f"run_{stamp}"
        # Keep the directory shape aligned with ACCEPTANCE.md from the first
        # commit so future tooling can rely on stable paths.
        for subdir in ["rosbag2", "policy_io", "captures", "configs"]:
            (run_dir / subdir).mkdir(parents=True, exist_ok=True)
        snapshot = {
            "created_at": datetime.now().isoformat(timespec="seconds"),
            "expected_stack": {
                "ubuntu": "22.04",
                "ros2": "humble",
                "mujoco": "TBD",
            },
        }
        (run_dir / "configs" / "system_snapshot.json").write_text(
            json.dumps(snapshot, indent=2) + "\n", encoding="utf-8"
        )
        return run_dir

    def write_event(self, event_type, payload):
        # JSONL gives append-only logs that are easy to inspect during early
        # integration and easy to replay into richer tooling later.
        record = {
            "time": self.get_clock().now().nanoseconds,
            "type": event_type,
            "payload": payload,
        }
        with self.event_log_path.open("a", encoding="utf-8") as handle:
            handle.write(json.dumps(record, sort_keys=True) + "\n")

    def publish_run_dir(self):
        msg = String()
        msg.data = str(self.run_dir)
        self.run_dir_pub.publish(msg)

    def on_safety_event(self, msg):
        self.write_event(
            "safety_event",
            {
                "level": int(msg.level),
                "code": msg.code,
                "message": msg.message,
                "abort_active": bool(msg.abort_active),
                "source": msg.source,
            },
        )

    def on_policy_status(self, msg):
        self.write_event(
            "policy_status",
            {
                "role": msg.policy_role,
                "policy": msg.policy_name,
                "loaded": bool(msg.loaded),
                "input_ready": bool(msg.input_ready),
                "missing_inputs": list(msg.missing_inputs),
                "latency_ms": float(msg.inference_latency_ms),
            },
        )

    def on_thruster_cmd(self, msg):
        self.write_event(
            "thruster_cmd",
            {
                "source": msg.source,
                "enable": bool(msg.enable),
                "normalized": [float(v) for v in msg.normalized],
            },
        )

    def on_arm_cmd(self, msg):
        self.write_event(
            "arm_cmd",
            {
                "source": msg.source,
                "enable": bool(msg.enable),
                "command_type": int(msg.command_type),
                "joint_names": list(msg.joint_names),
                "joint_targets": [float(v) for v in msg.joint_targets],
            },
        )

    def on_trajectory_target(self, msg):
        self.write_event(
            "trajectory_target",
            {
                "trajectory_type": msg.trajectory_type,
                "time_s": float(msg.time_s),
                "position": [
                    float(msg.target_pose.position.x),
                    float(msg.target_pose.position.y),
                    float(msg.target_pose.position.z),
                ],
                "linear_velocity": [
                    float(msg.target_twist.linear.x),
                    float(msg.target_twist.linear.y),
                    float(msg.target_twist.linear.z),
                ],
                "linear_acceleration": [
                    float(msg.target_accel.linear.x),
                    float(msg.target_accel.linear.y),
                    float(msg.target_accel.linear.z),
                ],
                "valid": bool(msg.valid),
            },
        )

    def on_tracking_status(self, msg):
        self.write_event(
            "tracking_status",
            {
                "trajectory_type": msg.trajectory_type,
                "time_s": float(msg.time_s),
                "valid": bool(msg.valid),
                "target_position": [
                    float(msg.target_position.x),
                    float(msg.target_position.y),
                    float(msg.target_position.z),
                ],
                "actual_position": [
                    float(msg.actual_position.x),
                    float(msg.actual_position.y),
                    float(msg.actual_position.z),
                ],
                "position_error_m": float(msg.position_error_m),
                "velocity_error_mps": float(msg.velocity_error_mps),
                "speed_mps": float(msg.speed_mps),
                "acceleration_mps2": float(msg.acceleration_mps2),
            },
        )


def main(args=None):
    rclpy.init(args=args)
    node = RunLogger()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
