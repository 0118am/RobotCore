"""Runtime blackboard node.

The blackboard publishes a compact JSON summary for UI and debugging tools. It
does not replace the source topics; it only makes common readiness checks easy.
"""

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from eup_interfaces.msg import ArmState, BodyState, PolicyStatus, SafetyEvent


class Blackboard(Node):
    """Aggregates high-level runtime state into one JSON status topic."""

    def __init__(self):
        super().__init__("blackboard")
        # Store only high-signal fields here. Detailed telemetry remains on the
        # original ROS topics and in rosbag2/event logs.
        self.state = {
            "body_state_valid": False,
            "arm_state_valid": False,
            "body_policy_ready": False,
            "arm_policy_ready": False,
            "abort_active": False,
        }
        self.pub = self.create_publisher(String, "/runtime/blackboard/state", 10)
        self.create_subscription(BodyState, "/robot/body_state", self.on_body, 10)
        self.create_subscription(ArmState, "/robot/arm_state", self.on_arm, 10)
        self.create_subscription(
            PolicyStatus, "/policy/body/status", self.on_policy, 10
        )
        self.create_subscription(PolicyStatus, "/policy/arm/status", self.on_policy, 10)
        self.create_subscription(SafetyEvent, "/safety/events", self.on_safety, 10)
        self.timer = self.create_timer(0.5, self.publish)

    def on_body(self, msg):
        self.state["body_state_valid"] = bool(msg.state_valid)
        self.state["depth_m"] = float(msg.depth_m)

    def on_arm(self, msg):
        self.state["arm_state_valid"] = bool(msg.state_valid)
        self.state["arm_joint_count"] = len(msg.joint_names)

    def on_policy(self, msg):
        # PolicyStatus carries missing input names, which is more useful for
        # operators than a single ready/not-ready flag.
        key = "body_policy_ready" if msg.policy_role == "body" else "arm_policy_ready"
        self.state[key] = bool(msg.input_ready and msg.loaded)
        self.state[f"{msg.policy_role}_missing_inputs"] = list(msg.missing_inputs)

    def on_safety(self, msg):
        self.state["abort_active"] = bool(msg.abort_active)
        self.state["last_safety_code"] = msg.code

    def publish(self):
        msg = String()
        # Sorting keys keeps repeated status output stable for tests and diffs.
        msg.data = json.dumps(self.state, sort_keys=True)
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Blackboard()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
