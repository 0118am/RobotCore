"""Arm policy ROS node.

The node mirrors BodyPolicyNode but targets manipulator observations and arm
action vectors, preserving separate body/arm policy lifecycles.
"""

import json
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String

from robotcore_interfaces.msg import ArmState, PolicyStatus
from robotcore_interfaces.srv import SetPolicy

from .action_decoder import decode_arm_joint_targets
from .observation_builder import ObservationBuilder
from .policy_manifest import load_policy_manifest
from .runners.factory import create_runner


class ArmPolicyNode(Node):
    """Runs the active arm policy and reports readiness through PolicyStatus."""

    def __init__(self):
        super().__init__("arm_policy_node")
        self.declare_parameter("policy_name", "dummy_arm_policy")
        self.declare_parameter("policy_path", "models/policies/dummy_arm_policy/policy.yaml")
        self.declare_parameter("publish_rate_hz", 10.0)

        self.role = "arm"
        self.io_path = None
        self.inference_count = 0
        # Use generic names until the real arm model and joint list are selected
        # in Phase 0.
        self.joint_names = [f"joint_{index + 1}" for index in range(6)]
        self.builder = ObservationBuilder(["/robot/arm_state"], max_age_ns=1_000_000_000)
        self.load_policy(
            self.get_parameter("policy_name").value,
            self.get_parameter("policy_path").value,
        )

        self.action_pub = self.create_publisher(Float32MultiArray, "/policy/arm/action", 10)
        self.status_pub = self.create_publisher(PolicyStatus, "/policy/arm/status", 10)
        self.create_subscription(ArmState, "/robot/arm_state", self.on_arm_state, 10)
        self.create_subscription(String, "/runtime/run_dir", self.on_run_dir, 10)
        self.create_service(SetPolicy, "/policy/arm/set_policy", self.on_set_policy)

        rate = float(self.get_parameter("publish_rate_hz").value)
        self.timer = self.create_timer(1.0 / max(rate, 0.1), self.tick)

    def load_policy(self, name, path):
        # Keep policy switching and startup loading on the same code path.
        self.manifest = load_policy_manifest(path, name, self.role)
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

    def on_arm_state(self, msg):
        stamp_ns = self.get_clock().now().nanoseconds
        # The incoming state owns the authoritative joint order once a real or
        # simulated arm backend is active.
        self.joint_names = list(msg.joint_names) or self.joint_names
        self.builder.update(
            "/robot/arm_state",
            stamp_ns,
            {
                "joint_names": list(msg.joint_names),
                "position": [float(v) for v in msg.position],
                "state_valid": bool(msg.state_valid),
            },
        )

    def on_run_dir(self, msg):
        path = Path(msg.data) / "policy_io" / "arm_policy_io.jsonl"
        path.parent.mkdir(parents=True, exist_ok=True)
        self.io_path = path

    def tick(self):
        now_ns = self.get_clock().now().nanoseconds
        missing = self.builder.missing_inputs(now_ns)
        input_ready = not missing
        latency_ms = 0.0

        if self.runner and input_ready:
            # Arm policy output is decoded against the current joint count so a
            # future 7-DoF arm can reuse this node.
            started = time.perf_counter()
            action = decode_arm_joint_targets(
                self.runner.run(self.builder.build()), len(self.joint_names)
            )
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
        # Large observations should be recorded through rosbag2; this file keeps
        # policy decisions searchable by run folder.
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
    node = ArmPolicyNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
