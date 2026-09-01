"""Body policy ROS node.

The node loads a policy manifest, tracks body-state readiness, runs inference,
and publishes the policy's direct T1..T8 action vector.
"""

import json
import time
from collections import deque
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray, String

from robotcore_interfaces.msg import (
    BodyState,
    PolicyStatus,
    ThrusterCommand,
    TrajectoryTarget,
)
from robotcore_interfaces.srv import SetPolicy

from .action_decoder import decode_thruster_action
from .observation_builder import ObservationBuilder
from .policy_manifest import PolicyManifest, load_policy_manifest
from .runners.factory import create_runner


class BodyPolicyNode(Node):
    """Runs the active body policy and reports readiness through PolicyStatus."""

    def __init__(self):
        super().__init__("body_policy_node")
        sensor_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.BEST_EFFORT
        )
        self.declare_parameter("policy_name", "dummy_body_policy")
        self.declare_parameter("policy_path", "models/policies/dummy_body_policy/policy.yaml")
        # Zero selects the control rate declared by the policy manifest.
        self.declare_parameter("publish_rate_hz", 0.0)

        self.role = "body"
        self.io_path = None
        self.inference_count = 0
        self.body_state_history = deque(maxlen=256)
        self.external_imu_history = deque(maxlen=256)
        self.last_input_ready = False
        self.last_runtime_error = ""
        # The builder is re-created when a manifest is loaded so policy-specific
        # inputs, such as Isaac trajectory targets, drive missing_inputs exactly.
        self.builder = ObservationBuilder(["/robot/body_state"], max_age_ns=1_000_000_000)
        self.load_policy(
            self.get_parameter("policy_name").value,
            self.get_parameter("policy_path").value,
        )

        self.action_pub = self.create_publisher(
            Float32MultiArray, "/policy/body/action", 1
        )
        self.status_pub = self.create_publisher(
            PolicyStatus, "/policy/body/status", 1
        )
        self.create_subscription(BodyState, "/robot/body_state", self.on_body_state, 1)
        self.create_subscription(
            Imu,
            "/sensors/external_imu",
            self.on_external_imu,
            sensor_qos,
        )
        self.create_subscription(
            TrajectoryTarget,
            "/runtime/trajectory_target",
            self.on_trajectory_target,
            1,
        )
        self.create_subscription(
            ThrusterCommand,
            "/control/thruster_cmd",
            self.on_thruster_command,
            1,
        )
        self.create_subscription(String, "/runtime/run_dir", self.on_run_dir, 1)
        self.create_service(SetPolicy, "/policy/body/set_policy", self.on_set_policy)

        self.timer = self.create_timer(1.0 / self.policy_rate_hz(), self.tick)

    def load_policy(self, name, path):
        # Runner construction is isolated here so the SetPolicy service and
        # startup path behave identically.
        previous_runner = getattr(self, "runner", None)
        if previous_runner is not None:
            previous_runner.close()
        self.runner = None
        self.load_error = ""
        self.body_state_history.clear()
        self.external_imu_history.clear()
        self.last_input_ready = False
        self.last_runtime_error = ""

        try:
            manifest = load_policy_manifest(path, name, self.role)
            required_inputs = manifest.input_schema or ["/robot/body_state"]
            max_input_age_s = float(
                manifest.isaac_contract.get("max_input_age_s", 1.0)
            )
            builder = ObservationBuilder(
                required_inputs,
                max_age_ns=max(1, int(max_input_age_s * 1_000_000_000)),
            )
            state_delay_ns = max(
                0,
                int(
                    float(manifest.isaac_contract.get("state_delay_s", 0.0))
                    * 1_000_000_000
                ),
            )
        except Exception as exc:
            self.manifest = PolicyManifest(
                name=str(name).strip() or "invalid_policy",
                role=self.role,
                runner="invalid",
                model_path=str(path),
            )
            self.builder = ObservationBuilder(
                ["/robot/body_state"], max_age_ns=1_000_000_000
            )
            self.state_delay_ns = 0
            self.load_error = f"policy manifest rejected: {exc}"
            self.get_logger().warn(self.load_error)
        else:
            self.manifest = manifest
            self.builder = builder
            self.state_delay_ns = state_delay_ns
            try:
                self.runner = create_runner(self.manifest)
            except Exception as exc:
                self.load_error = str(exc)
                self.get_logger().warn(self.load_error)

        if hasattr(self, "timer"):
            self.destroy_timer(self.timer)
            self.timer = self.create_timer(1.0 / self.policy_rate_hz(), self.tick)

    def policy_rate_hz(self):
        configured_rate = float(self.get_parameter("publish_rate_hz").value)
        if configured_rate > 0.0:
            return max(configured_rate, 0.1)
        contract_rate = float(
            self.manifest.isaac_contract.get("control_rate_hz", 60.0)
        )
        return max(contract_rate, 0.1)

    def on_set_policy(self, request, response):
        if request.activate:
            self.load_policy(request.policy_name, request.policy_path)
        response.accepted = self.runner is not None
        response.active_policy = self.manifest.name
        response.message = "loaded" if self.runner else self.load_error
        return response

    def on_body_state(self, msg):
        received_ns = self.get_clock().now().nanoseconds
        source_stamp_ns = (
            int(msg.header.stamp.sec) * 1_000_000_000
            + int(msg.header.stamp.nanosec)
        )
        value = {
            "state_valid": bool(msg.state_valid),
            "position_estimated": bool(msg.position_estimated),
            "linear_velocity_valid": bool(msg.linear_velocity_valid),
            "source_stamp_ns": source_stamp_ns,
            "pose": {
                "position": [
                    float(msg.pose.position.x),
                    float(msg.pose.position.y),
                    float(msg.pose.position.z),
                ],
                "orientation": [
                    float(msg.pose.orientation.w),
                    float(msg.pose.orientation.x),
                    float(msg.pose.orientation.y),
                    float(msg.pose.orientation.z),
                ],
            },
            "twist": {
                "linear": [
                    float(msg.twist.linear.x),
                    float(msg.twist.linear.y),
                    float(msg.twist.linear.z),
                ],
            },
        }
        self.builder.update(
            "/robot/body_state",
            received_ns,
            value,
        )
        self.body_state_history.append((source_stamp_ns, value))

    def on_external_imu(self, msg):
        received_ns = self.get_clock().now().nanoseconds
        source_stamp_ns = (
            int(msg.header.stamp.sec) * 1_000_000_000
            + int(msg.header.stamp.nanosec)
        )
        value = {
            "source_stamp_ns": source_stamp_ns,
            "orientation": [
                float(msg.orientation.w),
                float(msg.orientation.x),
                float(msg.orientation.y),
                float(msg.orientation.z),
            ],
            "angular_velocity": [
                float(msg.angular_velocity.x),
                float(msg.angular_velocity.y),
                float(msg.angular_velocity.z),
            ],
        }
        self.builder.update("/sensors/external_imu", received_ns, value)
        self.external_imu_history.append((source_stamp_ns, value))

    def on_thruster_command(self, msg):
        self.builder.update(
            "/control/thruster_cmd",
            self.get_clock().now().nanoseconds,
            {"action": [float(value) for value in msg.action]},
        )

    def on_trajectory_target(self, msg):
        stamp_ns = self.get_clock().now().nanoseconds
        # The target remains in world/map coordinates here. OnnxRunner converts
        # it into body-frame position error and velocity to match IsaacLab.
        self.builder.update(
            "/runtime/trajectory_target",
            stamp_ns,
            {
                "valid": bool(msg.valid),
                "trajectory_type": str(msg.trajectory_type),
                "time_s": float(msg.time_s),
                "pose": {
                    "position": [
                        float(msg.target_pose.position.x),
                        float(msg.target_pose.position.y),
                        float(msg.target_pose.position.z),
                    ],
                    "orientation": [
                        float(msg.target_pose.orientation.w),
                        float(msg.target_pose.orientation.x),
                        float(msg.target_pose.orientation.y),
                        float(msg.target_pose.orientation.z),
                    ],
                },
                "twist": {
                    "linear": [
                        float(msg.target_twist.linear.x),
                        float(msg.target_twist.linear.y),
                        float(msg.target_twist.linear.z),
                    ],
                    "angular": [
                        float(msg.target_twist.angular.x),
                        float(msg.target_twist.angular.y),
                        float(msg.target_twist.angular.z),
                    ],
                },
                "accel": {
                    "linear": [
                        float(msg.target_accel.linear.x),
                        float(msg.target_accel.linear.y),
                        float(msg.target_accel.linear.z),
                    ],
                    "angular": [
                        float(msg.target_accel.angular.x),
                        float(msg.target_accel.angular.y),
                        float(msg.target_accel.angular.z),
                    ],
                },
            },
        )

    def on_run_dir(self, msg):
        path = Path(msg.data) / "policy_io" / "body_policy_io.jsonl"
        path.parent.mkdir(parents=True, exist_ok=True)
        self.io_path = path

    def tick(self):
        now_ns = self.get_clock().now().nanoseconds
        missing = self.builder.missing_inputs(now_ns)
        observation = self.builder.build()
        if "/robot/body_state" in self.manifest.input_schema:
            delayed_body = self.delayed_sample(self.body_state_history, now_ns)
            if delayed_body is None:
                missing.append("/robot/body_state(delayed)")
            else:
                observation["/robot/body_state"] = delayed_body
                if not (
                    delayed_body.get("state_valid", False)
                    or delayed_body.get("position_estimated", False)
                ):
                    missing.append("/robot/body_state(localization_invalid)")
                if not delayed_body.get("linear_velocity_valid", False):
                    missing.append("/robot/body_state(linear_velocity_invalid)")
        if "/sensors/external_imu" in self.manifest.input_schema:
            delayed_imu = self.delayed_sample(self.external_imu_history, now_ns)
            if delayed_imu is None:
                missing.append("/sensors/external_imu(delayed)")
            else:
                observation["/sensors/external_imu"] = delayed_imu
        if "/runtime/trajectory_target" in self.manifest.input_schema:
            target = observation.get("/runtime/trajectory_target", {})
            if not target.get("valid", False):
                missing.append("/runtime/trajectory_target(invalid)")
        missing = list(dict.fromkeys(missing))
        input_ready = not missing
        latency_ms = 0.0

        if self.runner and input_ready:
            # Measure only runner+decoder time; ROS publish overhead is tracked
            # separately by middleware tooling if needed.
            started = time.perf_counter()
            try:
                action = decode_thruster_action(self.runner.run(observation))
                latency_ms = (time.perf_counter() - started) * 1000.0
                msg = Float32MultiArray()
                msg.data = action
                self.action_pub.publish(msg)
                self.inference_count += 1
                self.write_policy_io(observation, action)
                self.last_runtime_error = ""
            except Exception as exc:
                runtime_error = f"inference: {exc}"
                if runtime_error != self.last_runtime_error:
                    self.get_logger().error(runtime_error)
                self.last_runtime_error = runtime_error
                missing.append(runtime_error)
                input_ready = False
                self.runner.reset()

        if self.runner and not input_ready and self.last_input_ready:
            self.runner.reset()
        self.last_input_ready = input_ready

        self.publish_status(input_ready, missing, latency_ms)

    def delayed_sample(self, history, now_ns):
        if not history:
            return None
        cutoff_ns = int(now_ns) - self.state_delay_ns
        for source_ns, value in reversed(history):
            if source_ns <= cutoff_ns:
                return value
        return None

    def close_runner(self):
        if self.runner is not None:
            self.runner.close()

    def write_policy_io(self, observation, action):
        if not self.io_path:
            return
        # Log keys rather than full observation payloads; large image/depth data
        # should live in rosbag2.
        record = {
            "time": self.get_clock().now().nanoseconds,
            "policy": self.manifest.name,
            "observation_keys": sorted(observation.keys()),
            "action": action,
        }
        with self.io_path.open("a", encoding="utf-8") as handle:
            handle.write(json.dumps(record, sort_keys=True) + "\n")

    def publish_status(self, input_ready, missing, latency_ms):
        status = PolicyStatus()
        status.header.stamp = self.get_clock().now().to_msg()
        status.policy_role = self.role
        status.policy_name = self.manifest.name
        status.runner = self.manifest.runner
        status.model_path = self.manifest.model_path
        status.loaded = self.runner is not None
        status.input_ready = bool(input_ready)
        status.missing_inputs = list(missing)
        status.inference_latency_ms = float(latency_ms)
        status.inference_count = self.inference_count
        status.tick_interval_ms = 0.0
        status.deadline_miss_count = 0
        self.status_pub.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = BodyPolicyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close_runner()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
