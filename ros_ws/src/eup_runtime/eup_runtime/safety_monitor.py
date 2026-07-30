"""Safety monitor node.

The monitor owns the system abort latch and continuously publishes a zero
thruster command while abort is active. That gives every backend the same simple
contract: listen to /control/thruster_cmd and obey the latest safe command.
"""

import rclpy
from rclpy.node import Node

from eup_interfaces.msg import SafetyEvent, ThrusterCommand
from eup_interfaces.srv import TriggerAbort


class SafetyMonitor(Node):
    """Owns abort state and publishes zero thrusters while abort is active."""

    def __init__(self):
        super().__init__("safety_monitor")
        self.abort_active = False
        self.abort_reason = ""
        self.zero_publish_hz = 20.0

        self.event_pub = self.create_publisher(SafetyEvent, "/safety/events", 10)
        # Publish to the normal control topic so MuJoCo and real hardware use the
        # same safety path.
        self.thruster_pub = self.create_publisher(
            ThrusterCommand, "/control/thruster_cmd", 10
        )
        self.abort_srv = self.create_service(
            TriggerAbort, "/safety/abort", self.handle_abort
        )
        self.timer = self.create_timer(1.0 / self.zero_publish_hz, self.tick)

    def handle_abort(self, request, response):
        # The service acts as a latch: request.abort=True engages abort, and
        # False clears it for controlled recovery during tests.
        self.abort_active = bool(request.abort)
        self.abort_reason = request.reason or "operator_request"

        code = "ABORT_ACTIVE" if self.abort_active else "ABORT_CLEARED"
        message = self.abort_reason if self.abort_active else "Abort state cleared"
        self.publish_event(SafetyEvent.LEVEL_ABORT, code, message)

        response.accepted = True
        response.abort_active = self.abort_active
        response.message = message
        return response

    def tick(self):
        if self.abort_active:
            # Keep publishing while latched so late subscribers and hardware
            # bridges cannot miss the zero command.
            self.thruster_pub.publish(self.zero_thruster_command())

    def zero_thruster_command(self):
        msg = ThrusterCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.normalized = [0.0] * 8
        msg.enable = False
        msg.source = "safety_monitor"
        return msg

    def publish_event(self, level, code, message):
        event = SafetyEvent()
        event.header.stamp = self.get_clock().now().to_msg()
        event.level = level
        event.code = code
        event.message = message
        event.abort_active = self.abort_active
        event.source = "safety_monitor"
        self.event_pub.publish(event)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyMonitor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
