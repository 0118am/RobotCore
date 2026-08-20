"""Fail-closed operator and low-level-board safety monitor."""

import time

import rclpy
from rclpy.node import Node

from robotcore_interfaces.msg import BoardStatus, SafetyEvent
from robotcore_interfaces.srv import TriggerAbort


class SafetyMonitor(Node):
    """Owns the abort latch and emits a fail-closed heartbeat."""

    def __init__(self):
        super().__init__("safety_monitor")
        self.declare_parameter("publish_rate_hz", 10.0)
        # The bridge publishes BoardStatus at 10 Hz. Five periods avoid false trips from a few
        # delayed ROS callbacks while the MCU's independent command timeout
        # remains the actuator's hard communication deadline.
        self.declare_parameter("board_status_timeout_s", 0.50)

        self.operator_abort_active = False
        self.operator_abort_reason = ""
        self.abort_active = True
        self.board_status_ns = None
        self.board_connected = False
        self.board_heartbeat_ok = False
        self.board_failsafe_active = False

        self.event_pub = self.create_publisher(SafetyEvent, "/safety/events", 1)
        self.create_subscription(
            BoardStatus, "/hardware/board_status", self.on_board_status, 1
        )
        self.abort_srv = self.create_service(
            TriggerAbort, "/safety/abort", self.handle_abort
        )
        publish_rate_hz = max(
            1.0, float(self.get_parameter("publish_rate_hz").value)
        )
        self.timer = self.create_timer(1.0 / publish_rate_hz, self.tick)

    @staticmethod
    def steady_now_ns():
        """Return process-local monotonic time for status freshness."""

        return time.monotonic_ns()

    def on_board_status(self, message):
        # This board exposes only actual safety signals; voltage and temperature
        # fields do not exist because the hardware has no such sensors.
        self.board_connected = bool(message.connected)
        self.board_heartbeat_ok = bool(message.heartbeat_ok)
        self.board_failsafe_active = bool(message.failsafe_active)
        self.board_status_ns = self.steady_now_ns()

    def board_failure(self, now_ns):
        """Return the current board failure as ``(code, message)`` or None."""

        if self.board_status_ns is None:
            return ("BOARD_STATUS_MISSING", "low-level board status is missing")
        timeout_s = max(
            0.0, float(self.get_parameter("board_status_timeout_s").value)
        )
        age_s = (now_ns - self.board_status_ns) * 1e-9
        if age_s > timeout_s:
            return ("BOARD_STATUS_STALE", "low-level board status is stale")
        if not self.board_connected:
            return ("BOARD_DISCONNECTED", "low-level board is disconnected")
        if not self.board_heartbeat_ok:
            return ("BOARD_HEARTBEAT_LOST", "low-level board heartbeat is not healthy")
        if self.board_failsafe_active:
            return ("BOARD_FAILSAFE_ACTIVE", "low-level board failsafe is active")
        return None

    def update_abort_state(self, now_ns):
        """Combine dynamic board safety with the operator-owned abort latch."""

        board_failure = self.board_failure(now_ns)
        if board_failure is not None:
            self.abort_active = True
            return board_failure
        if self.operator_abort_active:
            self.abort_active = True
            return (
                "ABORT_ACTIVE",
                self.operator_abort_reason or "operator_request",
            )
        self.abort_active = False
        return ("ABORT_CLEAR", "Safety monitor healthy")

    def handle_abort(self, request, response):
        # Only the operator request is latched here. Board faults are live
        # inputs and cannot be cleared through this service.
        self.operator_abort_active = bool(request.abort)
        self.operator_abort_reason = (
            request.reason or "operator_request"
        ) if self.operator_abort_active else ""
        code, message = self.update_abort_state(self.steady_now_ns())
        if not self.abort_active:
            code = "ABORT_CLEARED"
            message = "Abort state cleared"
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
        code, message = self.update_abort_state(self.steady_now_ns())
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
