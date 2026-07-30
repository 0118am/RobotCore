"""Safety monitor node.

The monitor owns the system abort latch and publishes a heartbeat. The command
authority consumes that state and remains the only final thruster publisher.
"""

import rclpy
from rclpy.node import Node

from eup_interfaces.msg import SafetyEvent
from eup_interfaces.srv import TriggerAbort


class SafetyMonitor(Node):
    """Owns the abort latch and emits a fail-closed heartbeat."""

    def __init__(self):
        super().__init__("safety_monitor")
        self.abort_active = False
        self.abort_reason = ""
        self.zero_publish_hz = 20.0

        self.event_pub = self.create_publisher(SafetyEvent, "/safety/events", 10)
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
        self.publish_event(
            SafetyEvent.LEVEL_ABORT if self.abort_active else SafetyEvent.LEVEL_INFO,
            code,
            message,
        )

        response.accepted = True
        response.abort_active = self.abort_active
        response.message = message
        return response

    def tick(self):
        code = "ABORT_ACTIVE" if self.abort_active else "ABORT_CLEAR"
        message = self.abort_reason if self.abort_active else "Safety monitor healthy"
        self.publish_event(
            SafetyEvent.LEVEL_ABORT if self.abort_active else SafetyEvent.LEVEL_INFO,
            code,
            message,
        )

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
