"""Run logger node.

This node creates the acceptance-test run folder and writes lightweight JSONL
records for events and command/status streams. rosbag2 recording can be added
around the same run directory in a later phase.
"""

import json
import hashlib
import os
import shutil
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from eup_interfaces.msg import (
    ArmCommand,
    ControlAuthorityStatus,
    PidStatus,
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
        self.declare_parameter("pid_config_path", "src/eup_control/config/real_pool_pid.yaml")
        self.declare_parameter(
            "thruster_config_path", "src/eup_control/config/real_pool_thrusters.yaml"
        )
        self.declare_parameter("scenario_config_path", "src/eup_runtime/config/tracking_scenarios.yaml")
        self.declare_parameter(
            "safety_config_path", "src/eup_control/config/real_pool_safety.yaml"
        )
        root = Path(self.get_parameter("run_root").value)
        self.run_dir = self.create_run_dir(root)
        self.event_log_path = self.run_dir / "event_log.jsonl"
        self.last_safety_signature = None
        self.last_safety_log_ns = 0
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
        self.create_subscription(
            ControlAuthorityStatus,
            "/control/authority/status",
            self.on_authority_status,
            20,
        )
        self.create_subscription(PidStatus, "/control/pid/status", self.on_pid_status, 20)
        self.create_subscription(
            String,
            "/runtime/tracking_experiment/event",
            self.on_tracking_experiment_event,
            10,
        )
        self.snapshot_control_configs()

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

    def snapshot_control_configs(self):
        """Copy exact control inputs into the immutable run snapshot folder."""

        destination = self.run_dir / "configs"
        hashes = {}
        for parameter in (
            "pid_config_path",
            "thruster_config_path",
            "scenario_config_path",
            "safety_config_path",
        ):
            source = Path(str(self.get_parameter(parameter).value))
            if source.is_file():
                shutil.copy2(source, destination / source.name)
                hashes[source.name] = hashlib.sha256(source.read_bytes()).hexdigest()
        (destination / "control_config_hashes.json").write_text(
            json.dumps(hashes, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )

    def publish_run_dir(self):
        msg = String()
        msg.data = str(self.run_dir)
        self.run_dir_pub.publish(msg)

    def on_safety_event(self, msg):
        now_ns = self.get_clock().now().nanoseconds
        signature = (msg.code, msg.message, bool(msg.abort_active))
        if (
            signature == self.last_safety_signature
            and now_ns - self.last_safety_log_ns < 1_000_000_000
        ):
            return
        self.last_safety_signature = signature
        self.last_safety_log_ns = now_ns
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

    def on_tracking_experiment_event(self, msg):
        try:
            payload = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            payload = {"phase": "invalid", "message": str(msg.data)}
        self.write_event("tracking_experiment", payload)

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
                "orientation_wxyz": [
                    float(msg.target_pose.orientation.w),
                    float(msg.target_pose.orientation.x),
                    float(msg.target_pose.orientation.y),
                    float(msg.target_pose.orientation.z),
                ],
                "angular_velocity": [
                    float(msg.target_twist.angular.x),
                    float(msg.target_twist.angular.y),
                    float(msg.target_twist.angular.z),
                ],
                "angular_acceleration": [
                    float(msg.target_accel.angular.x),
                    float(msg.target_accel.angular.y),
                    float(msg.target_accel.angular.z),
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
                "orientation_error_rad": float(msg.orientation_error_rad),
                "velocity_error_mps": float(msg.velocity_error_mps),
                "angular_velocity_error_rps": float(msg.angular_velocity_error_rps),
                "speed_mps": float(msg.speed_mps),
                "acceleration_mps2": float(msg.acceleration_mps2),
                "target_orientation_wxyz": [
                    float(msg.target_orientation.w),
                    float(msg.target_orientation.x),
                    float(msg.target_orientation.y),
                    float(msg.target_orientation.z),
                ],
                "actual_orientation_wxyz": [
                    float(msg.actual_orientation.w),
                    float(msg.actual_orientation.x),
                    float(msg.actual_orientation.y),
                    float(msg.actual_orientation.z),
                ],
                "target_velocity_body": [
                    float(msg.target_velocity_body.x),
                    float(msg.target_velocity_body.y),
                    float(msg.target_velocity_body.z),
                ],
                "actual_velocity_body": [
                    float(msg.actual_velocity_body.x),
                    float(msg.actual_velocity_body.y),
                    float(msg.actual_velocity_body.z),
                ],
                "target_acceleration_body": [
                    float(msg.target_acceleration_body.x),
                    float(msg.target_acceleration_body.y),
                    float(msg.target_acceleration_body.z),
                ],
                "actual_acceleration_body": [
                    float(msg.actual_acceleration_body.x),
                    float(msg.actual_acceleration_body.y),
                    float(msg.actual_acceleration_body.z),
                ],
                "target_angular_velocity_body": [
                    float(msg.target_angular_velocity_body.x),
                    float(msg.target_angular_velocity_body.y),
                    float(msg.target_angular_velocity_body.z),
                ],
                "actual_angular_velocity_body": [
                    float(msg.actual_angular_velocity_body.x),
                    float(msg.actual_angular_velocity_body.y),
                    float(msg.actual_angular_velocity_body.z),
                ],
                "position_error_body": [
                    float(msg.position_error_body.x),
                    float(msg.position_error_body.y),
                    float(msg.position_error_body.z),
                ],
                "orientation_error_body": [
                    float(msg.orientation_error_body.x),
                    float(msg.orientation_error_body.y),
                    float(msg.orientation_error_body.z),
                ],
            },
        )

    def on_authority_status(self, msg):
        self.write_event(
            "control_authority_status",
            {
                "selected_source": msg.selected_source,
                "armed": bool(msg.armed),
                "abort_active": bool(msg.abort_active),
                "fault_latched": bool(msg.fault_latched),
                "fault_code": msg.fault_code,
                "message": msg.message,
                "candidate_age_s": float(msg.candidate_age_s),
                "body_state_age_s": float(msg.body_state_age_s),
                "target_age_s": float(msg.target_age_s),
                "command_limit": float(msg.command_limit),
                "localization_source": msg.localization_source,
                "pool_bounds_configured": bool(msg.pool_bounds_configured),
            },
        )

    def on_pid_status(self, msg):
        self.write_event(
            "pid_status",
            {
                "ready": bool(msg.ready),
                "producing_command": bool(msg.producing_command),
                "missing_inputs": list(msg.missing_inputs),
                "allocation_rank": int(msg.allocation_rank),
                "allocation_condition": float(msg.allocation_condition),
                "allocation_residual": float(msg.allocation_residual),
                "saturation_fraction": float(msg.saturation_fraction),
                "configuration_hash": msg.configuration_hash,
                "message": msg.message,
            },
        )


def main(args=None):
    rclpy.init(args=args)
    node = RunLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
