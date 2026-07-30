"""Single-writer thruster command authority for manual, PID, and future RL."""

from __future__ import annotations

import math
import numpy as np
import rclpy
from rclpy.node import Node

from eup_interfaces.msg import (
    BodyState,
    ControlAuthorityStatus,
    SafetyEvent,
    ThrusterCommand,
    TrajectoryTarget,
)
from eup_interfaces.srv import SetControlAuthority


class CommandAuthorityNode(Node):
    SOURCES = ("manual", "pid", "rl")
    EXPECTED_PRODUCERS = {
        "manual": "web_operator",
        "pid": "pid_controller",
        "rl": "rl_action_adapter",
    }

    def __init__(self):
        super().__init__("command_authority")
        self.declare_parameter("publish_rate_hz", 100.0)
        self.declare_parameter("candidate_timeout_s", 0.10)
        self.declare_parameter("state_timeout_s", 0.15)
        self.declare_parameter("target_timeout_s", 0.15)
        self.declare_parameter("safety_heartbeat_timeout_s", 0.25)
        self.declare_parameter("automatic_command_limit", 0.15)
        self.declare_parameter("automatic_slew_rate_per_s", 0.5)
        self.declare_parameter("allow_rl_hardware", False)
        self.declare_parameter("pool_bounds_configured", False)
        self.declare_parameter("pool_min_xyz", [0.0, 0.0, 0.0])
        self.declare_parameter("pool_max_xyz", [0.0, 0.0, 0.0])

        self.selected_source = "manual"
        self.armed = False
        self.abort_active = False
        self.fault_latched = False
        self.fault_code = ""
        self.message = "disarmed"
        self.candidates = {}
        self.candidate_ns = {}
        self.body = None
        self.body_ns = None
        self.target = None
        self.target_ns = None
        self.safety_ns = None
        self.absolute_localization_seen = False
        self.last_output = np.zeros(8, dtype=np.float64)
        self.last_tick_ns = self.get_clock().now().nanoseconds

        self.command_pub = self.create_publisher(
            ThrusterCommand, "/control/thruster_cmd", 10
        )
        self.status_pub = self.create_publisher(
            ControlAuthorityStatus, "/control/authority/status", 10
        )
        for source in self.SOURCES:
            self.create_subscription(
                ThrusterCommand,
                f"/control/candidates/{source}",
                lambda message, selected=source: self.on_candidate(selected, message),
                20,
            )
        self.create_subscription(BodyState, "/robot/body_state", self.on_body, 20)
        self.create_subscription(
            TrajectoryTarget, "/runtime/trajectory_target", self.on_target, 20
        )
        self.create_subscription(SafetyEvent, "/safety/events", self.on_safety, 20)
        self.create_service(
            SetControlAuthority, "/control/authority/set", self.on_set_authority
        )
        rate = max(1.0, float(self.get_parameter("publish_rate_hz").value))
        self.timer = self.create_timer(1.0 / rate, self.tick)

    def on_candidate(self, source, message):
        self.candidates[source] = message
        self.candidate_ns[source] = self.get_clock().now().nanoseconds

    def on_body(self, message):
        self.body = message
        self.body_ns = self.get_clock().now().nanoseconds
        if message.state_valid:
            self.absolute_localization_seen = True

    def on_target(self, message):
        self.target = message
        self.target_ns = self.get_clock().now().nanoseconds

    def on_safety(self, message):
        self.abort_active = bool(message.abort_active)
        self.safety_ns = self.get_clock().now().nanoseconds
        if self.abort_active:
            self.trip("ABORT_ACTIVE", message.message or "safety abort")

    def on_set_authority(self, request, response):
        requested_source = str(request.source or self.selected_source).lower()
        if requested_source not in self.SOURCES:
            return self.reject(response, f"unsupported source: {requested_source}")
        if self.armed and requested_source != self.selected_source:
            return self.reject(response, "control source can change only while disarmed")
        if requested_source == "rl" and not bool(
            self.get_parameter("allow_rl_hardware").value
        ):
            return self.reject(response, "RL hardware authority is disabled")
        if request.clear_fault:
            if self.abort_active:
                return self.reject(response, "clear /safety/abort before clearing authority fault")
            self.fault_latched = False
            self.fault_code = ""
            self.message = "fault cleared"

        if not self.armed:
            self.selected_source = requested_source
        if not request.arm:
            self.armed = False
            self.last_output[:] = 0.0
            self.message = "disarmed"
            return self.accept(response)
        reason = self.prearm_failure(self.get_clock().now().nanoseconds)
        if reason:
            return self.reject(response, reason)
        self.armed = True
        self.message = f"armed {self.selected_source}"
        return self.accept(response)

    def accept(self, response):
        response.accepted = True
        response.selected_source = self.selected_source
        response.armed = self.armed
        response.fault_latched = self.fault_latched
        response.message = self.message
        return response

    def reject(self, response, message):
        response.accepted = False
        response.selected_source = self.selected_source
        response.armed = self.armed
        response.fault_latched = self.fault_latched
        response.message = str(message)
        return response

    @staticmethod
    def age_s(timestamp_ns, now_ns):
        return math.inf if timestamp_ns is None else (now_ns - timestamp_ns) * 1e-9

    def prearm_failure(self, now_ns):
        if self.abort_active:
            return "safety abort is active"
        if self.fault_latched:
            return f"authority fault is latched: {self.fault_code}"
        if self.age_s(self.safety_ns, now_ns) > float(
            self.get_parameter("safety_heartbeat_timeout_s").value
        ):
            return "safety monitor heartbeat is missing"
        candidate = self.candidates.get(self.selected_source)
        if candidate is None or self.age_s(
            self.candidate_ns.get(self.selected_source), now_ns
        ) > float(self.get_parameter("candidate_timeout_s").value):
            return f"{self.selected_source} candidate is missing or stale"
        if candidate.source != self.EXPECTED_PRODUCERS[self.selected_source]:
            return f"unexpected candidate producer: {candidate.source}"
        if len(candidate.normalized) != 8 or not all(
            math.isfinite(float(value)) for value in candidate.normalized
        ):
            return "candidate command is not eight finite values"
        if not candidate.enable:
            return f"{self.selected_source} candidate is not ready"
        if self.selected_source in {"pid", "rl"}:
            return self.automatic_prearm_failure(now_ns)
        return ""

    def manual_idle_reason(self, now_ns):
        """Describe a normal manual dead-man gap without treating it as a fault."""

        if self.selected_source != "manual":
            return ""
        candidate = self.candidates.get("manual")
        if candidate is None or self.age_s(
            self.candidate_ns.get("manual"), now_ns
        ) > float(self.get_parameter("candidate_timeout_s").value):
            return "manual candidate is missing or stale"
        if not candidate.enable:
            return "manual candidate is not ready"
        return ""

    def automatic_prearm_failure(self, now_ns):
        if not bool(self.get_parameter("pool_bounds_configured").value):
            return "pool bounds are not configured"
        if self.body is None or self.age_s(self.body_ns, now_ns) > float(
            self.get_parameter("state_timeout_s").value
        ):
            return "body state is missing or stale"
        if self.target is None or self.age_s(self.target_ns, now_ns) > float(
            self.get_parameter("target_timeout_s").value
        ):
            return "trajectory target is missing or stale"
        if not self.absolute_localization_seen:
            return "absolute localization has not been observed"
        if not self.body.linear_velocity_valid:
            return "linear velocity is invalid"
        if not (self.body.state_valid or self.body.position_estimated):
            return "localization is invalid"
        if self.body.position_estimated and not str(
            self.body.localization_source
        ).startswith("ZED VIO"):
            return "estimated localization source is not allowed"
        if not self.target.valid:
            return "trajectory target is invalid"
        if not self.inside_pool(self.body.pose.position):
            return "vehicle is outside configured pool bounds"
        return ""

    def inside_pool(self, position):
        minimum = np.asarray(self.get_parameter("pool_min_xyz").value, dtype=np.float64)
        maximum = np.asarray(self.get_parameter("pool_max_xyz").value, dtype=np.float64)
        point = np.asarray([position.x, position.y, position.z], dtype=np.float64)
        return bool(
            minimum.shape == (3,)
            and maximum.shape == (3,)
            and np.all(np.isfinite(point))
            and np.all(minimum < maximum)
            and np.all(point >= minimum)
            and np.all(point <= maximum)
        )

    def trip(self, code, message):
        self.armed = False
        self.fault_latched = True
        self.fault_code = str(code)
        self.message = str(message)
        self.last_output[:] = 0.0

    def tick(self):
        now = self.get_clock().now()
        now_ns = now.nanoseconds
        dt = max(0.0, min(0.1, (now_ns - self.last_tick_ns) * 1e-9))
        self.last_tick_ns = now_ns
        manual_idle_reason = ""
        if self.armed:
            reason = self.prearm_failure(now_ns)
            if reason:
                # Releasing the browser/gamepad dead-man switch intentionally
                # disables the manual candidate.  That is a neutral idle state,
                # not a Tracking/control fault.  Keep manual authority armed so
                # the next fresh enabled candidate can resume immediately.
                manual_idle_reason = self.manual_idle_reason(now_ns)
                if reason == manual_idle_reason:
                    self.last_output[:] = 0.0
                    self.message = f"armed manual; neutral: {manual_idle_reason}"
                else:
                    self.trip("CONTROL_INPUT_INVALID", reason)
        if self.armed and not manual_idle_reason:
            candidate = np.asarray(
                self.candidates[self.selected_source].normalized, dtype=np.float64
            )
            if self.selected_source in {"pid", "rl"}:
                limit = abs(float(self.get_parameter("automatic_command_limit").value))
                candidate = np.clip(candidate, -limit, limit)
                slew = abs(float(self.get_parameter("automatic_slew_rate_per_s").value))
                delta = slew * dt
                candidate = np.clip(
                    candidate, self.last_output - delta, self.last_output + delta
                )
            self.last_output = np.clip(candidate, -1.0, 1.0)
            self.message = f"armed {self.selected_source}"
            self.publish_command(now, self.last_output, True)
        else:
            self.last_output[:] = 0.0
            self.publish_command(now, self.last_output, False)
        self.publish_status(now, now_ns)

    def publish_command(self, now, values, enable):
        message = ThrusterCommand()
        message.header.stamp = now.to_msg()
        message.header.frame_id = "base_link"
        message.normalized = [float(value) for value in values]
        message.enable = bool(enable)
        message.source = f"command_authority:{self.selected_source}"
        self.command_pub.publish(message)

    def publish_status(self, now, now_ns):
        status = ControlAuthorityStatus()
        status.header.stamp = now.to_msg()
        status.selected_source = self.selected_source
        status.armed = self.armed
        status.abort_active = self.abort_active
        status.fault_latched = self.fault_latched
        status.fault_code = self.fault_code
        status.message = self.message
        status.candidate_age_s = self.age_s(
            self.candidate_ns.get(self.selected_source), now_ns
        )
        status.body_state_age_s = self.age_s(self.body_ns, now_ns)
        status.target_age_s = self.age_s(self.target_ns, now_ns)
        status.command_limit = float(self.get_parameter("automatic_command_limit").value)
        status.command_slew_rate = float(
            self.get_parameter("automatic_slew_rate_per_s").value
        )
        status.localization_source = (
            str(self.body.localization_source) if self.body is not None else ""
        )
        status.pool_bounds_configured = bool(
            self.get_parameter("pool_bounds_configured").value
        )
        self.status_pub.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = CommandAuthorityNode()
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
