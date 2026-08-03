"""Single-writer thruster command authority for manual, PID, and future RL."""

from __future__ import annotations

import math
import time

import numpy as np
import rclpy
from rclpy.node import Node

from robotcore_interfaces.msg import (
    BodyState,
    ControlAuthorityStatus,
    SafetyEvent,
    ThrusterCommand,
    TrajectoryTarget,
)
from robotcore_interfaces.srv import SetControlAuthority


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
        # One producer-freshness window is used for every candidate source.
        # At the default 100 Hz producer rate this tolerates several delayed
        # frames without allowing a separate, long-lived manual command lease.
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
        self.arm_generation = 0
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
        self.active_source = "manual"
        # This latch records that a fresh, valid, enabled manual candidate has
        # actually owned the output.  A later explicit disabled candidate is a
        # deliberate RELEASE; every other loss of that stream is a fault.
        self.manual_command_was_active = False
        self.last_output = np.zeros(8, dtype=np.float64)
        self.last_tick_ns = self.steady_now_ns()

        self.command_pub = self.create_publisher(
            ThrusterCommand, "/control/thruster_cmd", 1
        )
        self.status_pub = self.create_publisher(
            ControlAuthorityStatus, "/control/authority/status", 1
        )
        for source in self.SOURCES:
            self.create_subscription(
                ThrusterCommand,
                f"/control/candidates/{source}",
                lambda message, selected=source: self.on_candidate(selected, message),
                1,
            )
        self.create_subscription(BodyState, "/robot/body_state", self.on_body, 1)
        self.create_subscription(
            TrajectoryTarget, "/runtime/trajectory_target", self.on_target, 1
        )
        self.create_subscription(SafetyEvent, "/safety/events", self.on_safety, 1)
        self.create_service(
            SetControlAuthority, "/control/authority/set", self.on_set_authority
        )
        rate = max(1.0, float(self.get_parameter("publish_rate_hz").value))
        self.timer = self.create_timer(1.0 / rate, self.tick)

    def on_candidate(self, source, message):
        self.candidates[source] = message
        self.candidate_ns[source] = self.steady_now_ns()

    def on_body(self, message):
        self.body = message
        self.body_ns = self.steady_now_ns()
        if message.state_valid:
            self.absolute_localization_seen = True

    def on_target(self, message):
        self.target = message
        self.target_ns = self.steady_now_ns()

    def on_safety(self, message):
        self.abort_active = bool(message.abort_active)
        self.safety_ns = self.steady_now_ns()
        if self.abort_active and self.armed:
            self.trip(message.code or "ABORT_ACTIVE", message.message or "safety abort")

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
            self.manual_command_was_active = False
            self.last_output[:] = 0.0
            self.message = "disarmed"
            return self.accept(response)
        reason = self.prearm_failure(self.steady_now_ns())
        if reason:
            return self.reject(response, reason)
        if not self.armed:
            self.arm_generation = (self.arm_generation + 1) & 0xFFFFFFFFFFFFFFFF
        self.armed = True
        if self.selected_source == "manual":
            # prearm_failure() just proved that this manual candidate is fresh,
            # valid, and enabled.  Latch it now so a loss before the first timer
            # tick still fails closed.
            self.manual_command_was_active = True
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
        self.message = str(message)
        response.accepted = False
        response.selected_source = self.selected_source
        response.armed = self.armed
        response.fault_latched = self.fault_latched
        response.message = str(message)
        return response

    @staticmethod
    def age_s(timestamp_ns, now_ns):
        return math.inf if timestamp_ns is None else (now_ns - timestamp_ns) * 1e-9

    @staticmethod
    def steady_now_ns():
        """Return process-local monotonic time for deadlines and freshness."""

        return time.monotonic_ns()

    def prearm_failure(self, now_ns):
        reason = self.common_authority_failure(now_ns)
        if reason:
            return reason
        candidate = self.candidates.get(self.selected_source)
        reason = self.candidate_failure(self.selected_source, candidate, now_ns)
        if reason:
            return reason
        if self.selected_source in {"pid", "rl"}:
            return self.automatic_prearm_failure(now_ns)
        return ""

    def common_authority_failure(self, now_ns):
        if self.abort_active:
            return "safety abort is active"
        if self.fault_latched:
            return f"authority fault is latched: {self.fault_code}"
        if self.age_s(self.safety_ns, now_ns) > float(
            self.get_parameter("safety_heartbeat_timeout_s").value
        ):
            return "safety monitor heartbeat is missing"
        return ""

    def candidate_failure(self, source, candidate, now_ns):
        reason = self.candidate_integrity_failure(source, candidate, now_ns)
        if reason:
            return reason
        if not candidate.enable:
            return f"{source} candidate is not ready"
        return ""

    def candidate_integrity_failure(self, source, candidate, now_ns):
        """Return failures that cannot be treated as an explicit RELEASE."""

        if candidate is None or self.age_s(
            self.candidate_ns.get(source), now_ns
        ) > float(self.get_parameter("candidate_timeout_s").value):
            return f"{source} candidate is missing or stale"
        if candidate.source != self.EXPECTED_PRODUCERS[source]:
            return f"unexpected candidate producer: {candidate.source}"
        try:
            values = [float(value) for value in candidate.normalized]
        except (TypeError, ValueError, OverflowError):
            return "candidate command is not eight finite values"
        if len(values) != 8 or not all(math.isfinite(value) for value in values):
            return "candidate command is not eight finite values"
        return ""

    def manual_candidate_state(self, now_ns):
        """Classify manual input as active, explicitly released, or lost."""

        candidate = self.candidates.get("manual")
        reason = self.candidate_integrity_failure("manual", candidate, now_ns)
        if reason:
            return "lost", reason
        if not candidate.enable:
            return "released", "manual candidate is not ready"
        return "active", ""

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
        self.manual_command_was_active = False
        self.fault_latched = True
        self.fault_code = str(code)
        self.message = str(message)
        self.last_output[:] = 0.0

    def tick(self):
        now = self.get_clock().now()
        now_ns = self.steady_now_ns()
        dt = max(0.0, min(0.1, (now_ns - self.last_tick_ns) * 1e-9))
        self.last_tick_ns = now_ns
        manual_idle_reason = ""
        manual_state, manual_state_reason = self.manual_candidate_state(now_ns)
        if manual_state == "released":
            # A fresh, structurally valid enable=false frame is the only event
            # allowed to hand control back to an armed PID/RL source.
            self.manual_command_was_active = False

        manual_link_lost = (
            self.armed
            and self.manual_command_was_active
            and manual_state == "lost"
        )
        # Manual input can override an already-armed automatic controller, but
        # it never bypasses the authority arm state.
        manual_override = (
            self.armed
            and self.selected_source != "manual"
            and manual_state == "active"
        )
        self.active_source = (
            "manual" if manual_override or manual_link_lost else self.selected_source
        )
        output_allowed = self.armed
        if manual_link_lost:
            self.trip(
                "MANUAL_LINK_LOST",
                f"manual control stream lost: {manual_state_reason}",
            )
            output_allowed = False
        elif output_allowed:
            # Manual and automatic candidates keep publishing independently.
            # A fresh enabled web/manual candidate means LB is held, so only
            # the common safety gates apply until LB is released. The selected
            # PID/RL controller remains armed and resumes automatically.
            reason = (
                self.common_authority_failure(now_ns)
                if manual_override
                else self.prearm_failure(now_ns)
            )
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
                    output_allowed = False
        if output_allowed and not manual_idle_reason:
            candidate = np.asarray(
                self.candidates[self.active_source].normalized, dtype=np.float64
            )
            if self.active_source in {"pid", "rl"}:
                limit = abs(float(self.get_parameter("automatic_command_limit").value))
                candidate = np.clip(candidate, -limit, limit)
                slew = abs(float(self.get_parameter("automatic_slew_rate_per_s").value))
                delta = slew * dt
                candidate = np.clip(
                    candidate, self.last_output - delta, self.last_output + delta
                )
            self.last_output = np.clip(candidate, -1.0, 1.0)
            self.message = (
                f"armed {self.selected_source}; manual LB override"
                if manual_override
                else f"armed {self.selected_source}"
            )
            if self.active_source == "manual":
                self.manual_command_was_active = True
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
        message.armed = bool(self.armed)
        message.arm_generation = int(self.arm_generation)
        message.source = f"command_authority:{self.active_source}"
        self.command_pub.publish(message)

    def publish_status(self, now, now_ns):
        status = ControlAuthorityStatus()
        status.header.stamp = now.to_msg()
        status.selected_source = self.selected_source
        status.armed = self.armed
        status.arm_generation = int(self.arm_generation)
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
