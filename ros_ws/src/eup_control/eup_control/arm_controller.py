"""Arm controller node.

The controller converts arm-policy vectors into the stable ArmCommand message.
Driver-specific joint limits and transport stay outside this node for now.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from eup_interfaces.msg import ArmCommand


class ArmController(Node):
    """Routes arm policy vectors into the arm command contract."""

    def __init__(self):
        super().__init__("arm_controller")
        self.joint_names = [f"joint_{index + 1}" for index in range(6)]
        self.pub = self.create_publisher(ArmCommand, "/control/arm_cmd", 10)
        self.create_subscription(Float32MultiArray, "/policy/arm/action", self.on_action, 10)

    def on_action(self, msg):
        command = ArmCommand()
        command.header.stamp = self.get_clock().now().to_msg()
        command.header.frame_id = "arm_base"
        # Phase 1 uses joint targets. EE-delta and joint-delta modes are already
        # represented in the interface for later arm-policy experiments.
        command.command_type = ArmCommand.COMMAND_JOINT_TARGET
        command.joint_names = self.joint_names
        values = [float(v) for v in list(msg.data)[: len(self.joint_names)]]
        # Maintain a fixed joint vector length for the generic 6-DoF placeholder.
        command.joint_targets = values + [0.0] * (len(self.joint_names) - len(values))
        command.joint_deltas = []
        command.enable = True
        command.source = "arm_controller"
        self.pub.publish(command)


def main(args=None):
    rclpy.init(args=args)
    node = ArmController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
