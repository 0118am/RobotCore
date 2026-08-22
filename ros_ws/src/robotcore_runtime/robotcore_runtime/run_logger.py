"""Run logger node.

Each tracking task gets one run folder, one configuration snapshot, a compact
JSONL summary, and one full-rate rosbag2 recording.
"""

import json
import hashlib
import math
import os
import shutil
import signal
import subprocess
import time
from datetime import datetime
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from robotcore_interfaces.msg import (
    ControlAuthorityStatus,
    LocalizationStatus,
    PidStatus,
    PolicyStatus,
    SafetyEvent,
    ThrusterCommand,
    TrackingStatus,
    TrajectoryTarget,
)
from robotcore_interfaces.srv import StartRun, StopRun


def finite_or_none(value):
    value = float(value)
    return value if math.isfinite(value) else None


class RunLogger(Node):
    """Creates the run folder and writes lightweight JSONL event records."""

    def __init__(self):
        super().__init__("run_logger")
        # ROBOTCORE_RUN_ROOT lets launch files keep log output in the workspace while
        # still allowing deployment scripts to redirect logs to mounted storage.
        self.declare_parameter("run_root", os.environ.get("ROBOTCORE_RUN_ROOT", "data/robotcore_runs"))
        self.declare_parameter("pid_config_path", "src/robotcore_control/config/real_pool_pid.yaml")
        self.declare_parameter(
            "thruster_config_path", "src/robotcore_control/config/real_pool_thrusters.yaml"
        )
        self.declare_parameter(
            "safety_config_path", "src/robotcore_control/config/real_pool_safety.yaml"
        )
        self.declare_parameter(
            "task_config_dir", "src/robotcore_runtime/config/tasks"
        )
        self.declare_parameter(
            "record_topics_path", "src/robotcore_runtime/config/tasks/record_topics.json"
        )
        self.declare_parameter("rosbag_executable", "/opt/ros/humble/bin/ros2")
        # JSONL is an operator-readable summary, not the full-rate transport
        # recording.  Bound repeated status streams here and leave lossless
        # capture to rosbag2 so logging cannot compete with control callbacks.
        self.declare_parameter("thruster_log_rate_hz", 20.0)
        self.declare_parameter("trajectory_log_rate_hz", 20.0)
        self.declare_parameter("tracking_log_rate_hz", 20.0)
        self.declare_parameter("authority_log_rate_hz", 10.0)
        self.declare_parameter("pid_log_rate_hz", 10.0)
        self.declare_parameter("flush_interval_s", 0.25)
        self.run_root = Path(self.get_parameter("run_root").value)
        self.run_dir: Path | None = None
        self.event_log_handle = None
        self.rosbag_process = None
        self.rosbag_output_handle = None
        self.last_stream_log_ns = {}
        self.stream_log_rates = {
            "thruster_cmd": float(self.get_parameter("thruster_log_rate_hz").value),
            "trajectory_target": float(
                self.get_parameter("trajectory_log_rate_hz").value
            ),
            "tracking_status": float(self.get_parameter("tracking_log_rate_hz").value),
            "control_authority_status": float(
                self.get_parameter("authority_log_rate_hz").value
            ),
            "pid_status": float(self.get_parameter("pid_log_rate_hz").value),
        }
        self.last_safety_signature = None
        self.last_safety_log_ns = 0
        self.last_localization_log_ns = 0
        # Policy nodes subscribe to this topic so all policy_io files land in
        # the same run folder without sharing process-local state.
        run_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.run_dir_pub = self.create_publisher(String, "/runtime/run_dir", run_qos)
        summary_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        self.create_subscription(SafetyEvent, "/safety/events", self.on_safety_event, 20)
        self.create_subscription(
            PolicyStatus, "/policy/body/status", self.on_policy_status, 20
        )
        self.create_subscription(
            ThrusterCommand, "/control/thruster_cmd", self.on_thruster_cmd, summary_qos
        )
        self.create_subscription(
            TrajectoryTarget, "/runtime/trajectory_target", self.on_trajectory_target, summary_qos
        )
        self.create_subscription(
            TrackingStatus, "/runtime/tracking_status", self.on_tracking_status, summary_qos
        )
        self.create_subscription(
            ControlAuthorityStatus,
            "/control/authority/status",
            self.on_authority_status,
            summary_qos,
        )
        self.create_subscription(PidStatus, "/control/pid/status", self.on_pid_status, summary_qos)
        self.create_subscription(
            LocalizationStatus,
            "/localization/status",
            self.on_localization_status,
            summary_qos,
        )
        self.create_service(StartRun, "/runtime/run/start", self.on_start_run)
        self.create_service(StopRun, "/runtime/run/stop", self.on_stop_run)
        self.timer = self.create_timer(1.0, self.publish_run_dir)
        flush_interval = max(
            0.05, float(self.get_parameter("flush_interval_s").value)
        )
        self.flush_timer = self.create_timer(flush_interval, self.flush_event_log)

    def create_run_dir(self, root, task_name):
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        run_dir = root / f"run_{stamp}_{task_name}"
        # Keep the directory shape aligned with ACCEPTANCE.md from the first
        # commit so future tooling can rely on stable paths.
        for subdir in ["rosbag2", "policy_io", "captures", "configs"]:
            (run_dir / subdir).mkdir(parents=True, exist_ok=True)
        snapshot = {
            "created_at": datetime.now().isoformat(timespec="seconds"),
            "expected_stack": {
                "ubuntu": "22.04",
                "ros2": "humble",
            },
        }
        (run_dir / "configs" / "system_snapshot.json").write_text(
            json.dumps(snapshot, indent=2) + "\n", encoding="utf-8"
        )
        return run_dir

    def on_start_run(self, request, response):
        if self.event_log_handle is not None:
            response.success = False
            response.run_dir = str(self.run_dir)
            response.message = "a tracking run is already active"
            return response

        self.run_dir = self.create_run_dir(self.run_root, request.task_name)
        self.event_log_handle = (self.run_dir / "event_log.jsonl").open(
            "a", encoding="utf-8", buffering=64 * 1024
        )
        self.snapshot_control_configs(request.task_name)
        self.start_rosbag()
        self.write_event(
            "run_started",
            {
                "run_dir": str(self.run_dir),
                "rosbag2_dir": str(self.run_dir / "rosbag2" / "tracking"),
            },
            flush=True,
        )
        self.write_event(
            "tracking_experiment",
            {
                "phase": "start",
                "scenario": request.task_name,
                "controller": request.controller,
                "duration_s": float(request.duration_s),
                "success": True,
                "message": "started",
            },
            flush=True,
        )
        self.publish_run_dir()
        response.success = True
        response.run_dir = str(self.run_dir)
        response.message = "run directory and rosbag recording started"
        self.get_logger().info(f"Run folder: {self.run_dir}")
        return response

    def on_stop_run(self, request, response):
        run_dir = str(self.run_dir)
        self.write_event(
            "tracking_experiment",
            {
                "phase": "end",
                "success": bool(request.success),
                "message": request.message,
            },
            flush=True,
        )
        self.stop_rosbag()
        self.event_log_handle.close()
        self.event_log_handle = None
        self.run_dir = None
        response.accepted = True
        response.run_dir = run_dir
        response.message = "run log and rosbag recording stopped"
        return response

    def start_rosbag(self):
        topic_document = json.loads(
            Path(str(self.get_parameter("record_topics_path").value)).read_text(
                encoding="utf-8"
            )
        )
        output = self.run_dir / "rosbag2" / "tracking"
        command = [
            str(self.get_parameter("rosbag_executable").value),
            "bag",
            "record",
            "--storage",
            "sqlite3",
            "--output",
            str(output),
            *[str(topic) for topic in topic_document["topics"]],
        ]
        self.rosbag_output_handle = (self.run_dir / "rosbag2.log").open(
            "w", encoding="utf-8"
        )
        self.rosbag_process = subprocess.Popen(
            command,
            stdout=self.rosbag_output_handle,
            stderr=subprocess.STDOUT,
            text=True,
        )
        time.sleep(0.5)
        if self.rosbag_process.poll() is not None:
            raise RuntimeError("rosbag2 recorder exited during startup")

    def stop_rosbag(self):
        self.rosbag_process.send_signal(signal.SIGINT)
        self.rosbag_process.wait(timeout=10.0)
        self.rosbag_process = None
        self.rosbag_output_handle.close()
        self.rosbag_output_handle = None

    def write_event(self, event_type, payload, *, flush=False):
        # JSONL gives append-only logs that are easy to inspect during early
        # integration and easy to replay into richer tooling later.
        if self.event_log_handle is None:
            return
        record = {
            "time": self.get_clock().now().nanoseconds,
            "type": event_type,
            "payload": payload,
        }
        self.event_log_handle.write(
            json.dumps(record, sort_keys=True, separators=(",", ":")) + "\n"
        )
        if flush:
            self.event_log_handle.flush()

    def stream_log_due(self, event_type):
        """Use monotonic time to bound repeated summaries across ROS clock jumps."""

        rate_hz = self.stream_log_rates[event_type]
        if rate_hz <= 0.0:
            return False
        now_ns = time.monotonic_ns()
        previous_ns = self.last_stream_log_ns.get(event_type)
        period_ns = max(1, int(1e9 / rate_hz))
        if previous_ns is not None and 0 <= now_ns - previous_ns < period_ns:
            return False
        self.last_stream_log_ns[event_type] = now_ns
        return True

    def flush_event_log(self):
        if self.event_log_handle is not None:
            self.event_log_handle.flush()

    def snapshot_control_configs(self, task_name):
        """Copy exact control inputs into the immutable run snapshot folder."""

        destination = self.run_dir / "configs"
        hashes = {}
        for parameter in (
            "pid_config_path",
            "thruster_config_path",
            "safety_config_path",
        ):
            source = Path(str(self.get_parameter(parameter).value))
            if source.is_file():
                shutil.copy2(source, destination / source.name)
                hashes[source.name] = hashlib.sha256(source.read_bytes()).hexdigest()
        task_path = self.task_path(task_name)
        shutil.copy2(task_path, destination / task_path.name)
        hashes[task_path.name] = hashlib.sha256(task_path.read_bytes()).hexdigest()
        (destination / "control_config_hashes.json").write_text(
            json.dumps(hashes, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )

    def task_path(self, task_name):
        """Resolve the same managed-or-installed task document used to execute it."""

        filename = f"{task_name}.json"
        path = Path(str(self.get_parameter("task_config_dir").value)) / filename
        if path.is_file():
            return path
        return (
            Path(get_package_share_directory("robotcore_runtime"))
            / "config"
            / "tasks"
            / filename
        )

    def publish_run_dir(self):
        if self.run_dir is None:
            return
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
            flush=bool(msg.abort_active),
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
        if not self.stream_log_due("thruster_cmd"):
            return
        self.write_event(
            "thruster_cmd",
            {
                "source": msg.source,
                "enable": bool(msg.enable),
                "normalized": [float(v) for v in msg.normalized],
            },
        )

    def on_trajectory_target(self, msg):
        if not self.stream_log_due("trajectory_target"):
            return
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
        if not self.stream_log_due("tracking_status"):
            return
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
        if not self.stream_log_due("control_authority_status"):
            return
        self.write_event(
            "control_authority_status",
            {
                "selected_source": msg.selected_source,
                "armed": bool(msg.armed),
                "abort_active": bool(msg.abort_active),
                "fault_latched": bool(msg.fault_latched),
                "fault_code": msg.fault_code,
                "message": msg.message,
                "selected_command_age_s": float(msg.selected_command_age_s),
                "body_state_age_s": float(msg.body_state_age_s),
                "target_age_s": float(msg.target_age_s),
                "command_limit": float(msg.command_limit),
                "localization_source": msg.localization_source,
            },
        )

    def on_pid_status(self, msg):
        if not self.stream_log_due("pid_status"):
            return
        self.write_event(
            "pid_status",
            {
                "ready": bool(msg.ready),
                "producing_command": bool(msg.producing_command),
                "missing_inputs": list(msg.missing_inputs),
                "body_state_age_s": float(msg.body_state_age_s),
                "imu_age_s": float(msg.imu_age_s),
                "target_age_s": float(msg.target_age_s),
                "allocation_rank": int(msg.allocation_rank),
                "allocation_condition": float(msg.allocation_condition),
                "allocation_residual": float(msg.allocation_residual),
                "saturation_fraction": float(msg.saturation_fraction),
                "configuration_hash": msg.configuration_hash,
                "message": msg.message,
            },
        )

    def on_localization_status(self, msg):
        """Persist localisation health at 1 Hz without growing logs at filter rate."""

        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.last_localization_log_ns < 1_000_000_000:
            return
        self.last_localization_log_ns = now_ns
        diagonal_indices = (0, 7, 14, 21, 28, 35)
        pose_diagonal = [
            float(msg.pose_covariance[index]) for index in diagonal_indices
        ]
        twist_diagonal = [
            float(msg.twist_covariance[index]) for index in diagonal_indices
        ]
        self.write_event(
            "localization_status",
            {
                "localization_source": msg.localization_source,
                "vio_fresh": bool(msg.vio_fresh),
                "tag_fresh": bool(msg.tag_fresh),
                "tag_consistent": bool(msg.tag_consistent),
                "absolute_fix_valid": bool(msg.absolute_fix_valid),
                "position_estimated": bool(msg.position_estimated),
                "tag_observation_class": msg.tag_observation_class,
                "rejection_reason": msg.rejection_reason,
                "detected_tag_count": int(msg.detected_tag_count),
                "mapped_tag_count": int(msg.mapped_tag_count),
                "inlier_tag_count": int(msg.inlier_tag_count),
                "tag_reprojection_rms_px": finite_or_none(
                    msg.tag_reprojection_rms_px
                ),
                "minimum_tag_edge_px": finite_or_none(msg.minimum_tag_edge_px),
                "tag_pose_published": bool(msg.tag_pose_published),
                "apriltag_rejection_reason": msg.apriltag_rejection_reason,
                "vio_age_s": finite_or_none(msg.vio_age_s),
                "tag_age_s": finite_or_none(msg.tag_age_s),
                "absolute_fix_age_s": finite_or_none(msg.absolute_fix_age_s),
                "apriltag_frame_age_s": finite_or_none(
                    msg.apriltag_frame_age_s
                ),
                "vio_rate_hz": float(msg.vio_rate_hz),
                "tag_rate_hz": float(msg.tag_rate_hz),
                "body_state_rate_hz": float(msg.body_state_rate_hz),
                "apriltag_frame_rate_hz": float(msg.apriltag_frame_rate_hz),
                "vio_transport_delay_s": finite_or_none(msg.vio_transport_delay_s),
                "tag_transport_delay_s": finite_or_none(msg.tag_transport_delay_s),
                "tag_vio_translation_residual_m": finite_or_none(
                    msg.tag_vio_translation_residual_m
                ),
                "tag_vio_angle_residual_deg": finite_or_none(
                    msg.tag_vio_angle_residual_deg
                ),
                "pose_covariance_diagonal": pose_diagonal,
                "twist_covariance_diagonal": twist_diagonal,
            },
        )

    def destroy_node(self):
        handle = getattr(self, "event_log_handle", None)
        if getattr(self, "rosbag_process", None) is not None:
            self.stop_rosbag()
        if handle is not None and not handle.closed:
            handle.close()
        return super().destroy_node()


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
