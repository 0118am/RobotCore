"""Thruster allocator node.

The initial allocator passes an 8-element body-policy vector through as
normalized thruster commands. A real 6DoF wrench allocator can replace this
logic while preserving /control/thruster_cmd.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from robotcore_interfaces.msg import ThrusterCommand


def clamp(value):
    # The command contract is normalized [-1, 1] even if a policy runner emits a
    # slightly out-of-range value.
    return max(-1.0, min(1.0, float(value)))


class ThrusterAllocator(Node):
    """Maps body policy actions into the 8-thruster command contract."""

    def __init__(self):
        super().__init__("thruster_allocator")
        self.pub = self.create_publisher(ThrusterCommand, "/control/thruster_cmd", 10)
        self.create_subscription(
            Float32MultiArray, "/policy/body/action", self.on_action, 10
        )

    def on_action(self, msg):
        command = ThrusterCommand()
        command.header.stamp = self.get_clock().now().to_msg()
        command.header.frame_id = "base_link"
        values = [clamp(v) for v in list(msg.data)[:8]]
        # Pad short vectors to exactly eight channels so downstream backends
        # never need to infer actuator count.
        command.normalized = values + [0.0] * (8 - len(values))
        command.enable = True
        command.source = "thruster_allocator"
        self.pub.publish(command)


def main(args=None):
    rclpy.init(args=args)
    node = ThrusterAllocator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
