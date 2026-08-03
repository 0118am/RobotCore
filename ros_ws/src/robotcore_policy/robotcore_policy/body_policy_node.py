"""Body policy ROS node.

The node loads a policy manifest, tracks body-state readiness, runs inference,
and publishes a neutral action vector contract for `robotcore_control` to decode.
"""

import json
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String

from robotcore_interfaces.msg import BodyState, PolicyStatus, TrajectoryTarget
from robotcore_interfaces.srv import SetPolicy

from .action_decoder import decode_thruster_action
from .observation_builder import ObservationBuilder
from .policy_manifest import load_policy_manifest
from .runners.factory import create_runner


class BodyPolicyNode(Node):
    """Runs the active body policy and reports readiness through PolicyStatus."""

    def __init__(self):
        super().__init__("body_policy_node")
        self.declare_parameter("policy_name", "dummy_body_policy")
        self.declare_parameter("policy_path", "models/policies/dummy_body_policy/policy.yaml")
        # Isaac deployment contract: 60 Hz action updates (dt = 1/60 s).
        self.declare_parameter("publish_rate_hz", 60.0)

        self.role = "body"
        self.io_path = None
        self.inference_count = 0
        # The builder is re-created when a manifest is loaded so policy-specific
        # inputs, such as Isaac trajectory targets, drive missing_inputs exactly.
        self.builder = ObservationBuilder(["/robot/body_state"], max_age_ns=1_000_000_000)
        self.load_policy(
            self.get_parameter("policy_name").value,
            self.get_parameter("policy_path").value,
        )

        self.action_pub = self.create_publisher(
            Float32MultiArray, "/policy/body/action", 10
        )
        self.status_pub = self.create_publisher(
            PolicyStatus, "/policy/body/status", 10
        )
        self.create_subscription(BodyState, "/robot/body_state", self.on_body_state, 10)
        self.create_subscription(
            TrajectoryTarget,
            "/runtime/trajectory_target",
            self.on_trajectory_target,
            10,
        )
        self.create_subscription(String, "/runtime/run_dir", self.on_run_dir, 10)
        self.create_service(SetPolicy, "/policy/body/set_policy", self.on_set_policy)

        rate = float(self.get_parameter("publish_rate_hz").value)
        self.timer = self.create_timer(1.0 / max(rate, 0.1), self.tick)

    def load_policy(self, name, path):
        # Runner construction is isolated here so the SetPolicy service and
        # startup path behave identically.
        self.manifest = load_policy_manifest(path, name, self.role)
        required_inputs = self.manifest.input_schema or ["/robot/body_state"]
        self.builder = ObservationBuilder(required_inputs, max_age_ns=1_000_000_000)
        self.runner = None
        self.load_error = ""
        try:
            self.runner = create_runner(self.manifest)
        except Exception as exc:
            self.load_error = str(exc)
            self.get_logger().warn(self.load_error)

    def on_set_policy(self, request, response):
        if request.activate:
            self.load_policy(request.policy_name, request.policy_path)
        response.accepted = self.runner is not None
        response.active_policy = self.manifest.name
        response.message = "loaded" if self.runner else self.load_error
        return response

    def on_body_state(self, msg):
        stamp_ns = self.get_clock().now().nanoseconds
        # Store pose/twist fields needed by both simple body policies and the
        # IsaacLab 20-D trajectory observation builder.
        self.builder.update(
            "/robot/body_state",
            stamp_ns,
            {
                "state_valid": bool(msg.state_valid),
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
                    "angular": [
                        float(msg.twist.angular.x),
                        float(msg.twist.angular.y),
                        float(msg.twist.angular.z),
                    ],
                },
            },
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
        input_ready = not missing
        latency_ms = 0.0

        if self.runner and input_ready:
            # Measure only runner+decoder time; ROS publish overhead is tracked
            # separately by middleware tooling if needed.
            started = time.perf_counter()
            action = decode_thruster_action(self.runner.run(self.builder.build()))
            latency_ms = (time.perf_counter() - started) * 1000.0
            msg = Float32MultiArray()
            msg.data = action
            self.action_pub.publish(msg)
            self.inference_count += 1
            self.write_policy_io(self.builder.build(), action)

        self.publish_status(input_ready, missing, latency_ms)

    def write_policy_io(self, observation, action):
        if not self.io_path:
            return
        # Log keys rather than full observation payloads for the initial
        # skeleton; large image/depth data should live in rosbag2.
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
        self.status_pub.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = BodyPolicyNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
