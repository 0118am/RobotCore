"""Adapt policy vectors to the isolated future-RL command candidate."""

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from eup_interfaces.msg import ThrusterCommand

from .thruster_allocation import ThrusterAllocator


class RlActionAdapter(Node):
    def __init__(self):
        super().__init__("rl_action_adapter")
        self.declare_parameter("allow_actions", False)
        self.declare_parameter("policy_layout_hash", "")
        self.declare_parameter(
            "thruster_config_path",
            "src/eup_control/config/real_pool_thrusters.yaml",
        )
        self.hardware_layout_hash = ""
        self.layout_measured = False
        try:
            allocator = ThrusterAllocator.from_yaml(
                str(self.get_parameter("thruster_config_path").value)
            )
            self.hardware_layout_hash = allocator.config_hash
            self.layout_measured = bool(allocator.measured)
        except Exception as exc:
            self.get_logger().error(f"RL layout configuration rejected: {exc}")
        self.publisher = self.create_publisher(
            ThrusterCommand, "/control/candidates/rl", 10
        )
        self.create_subscription(
            Float32MultiArray, "/policy/body/action", self.on_action, 10
        )

    def on_action(self, message):
        values = list(message.data)
        policy_layout_hash = str(self.get_parameter("policy_layout_hash").value)
        layout_matches = (
            self.layout_measured
            and bool(policy_layout_hash)
            and policy_layout_hash == self.hardware_layout_hash
        )
        valid = (
            bool(self.get_parameter("allow_actions").value)
            and layout_matches
            and len(values) == 8
            and all(math.isfinite(float(value)) for value in values)
        )
        command = ThrusterCommand()
        command.header.stamp = self.get_clock().now().to_msg()
        command.header.frame_id = "base_link"
        command.normalized = (
            [max(-1.0, min(1.0, float(value))) for value in values]
            if valid
            else [0.0] * 8
        )
        command.enable = valid
        command.source = "rl_action_adapter"
        self.publisher.publish(command)


def main(args=None):
    rclpy.init(args=args)
    node = RlActionAdapter()
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
