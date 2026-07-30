"""Six-DoF cascaded pose/velocity PID with integrated thrust allocation."""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import WrenchStamped
from rclpy.node import Node
import yaml

from eup_interfaces.msg import (
    BodyState,
    ControlAuthorityStatus,
    PidStatus,
    ThrusterCommand,
    TrajectoryTarget,
)

from .control_math import (
    PidGains,
    SixAxisPid,
    normalize_quaternion,
    quaternion_apply,
    quaternion_conjugate,
    quaternion_error_body,
    vec,
)
from .thruster_allocation import ThrusterAllocator


def quaternion_from_message(message):
    return normalize_quaternion([message.w, message.x, message.y, message.z])


class SixDofPidNode(Node):
    """Convert full-pose trajectory targets to a PID command candidate."""

    def __init__(self):
        super().__init__("six_dof_pid_controller")
        self.declare_parameter("control_rate_hz", 60.0)
        self.declare_parameter("max_input_age_s", 0.15)
        self.declare_parameter("pid_config_path", "src/eup_control/config/real_pool_pid.yaml")
        self.declare_parameter(
            "thruster_config_path", "src/eup_control/config/real_pool_thrusters.yaml"
        )
        self.body = None
        self.body_ns = None
        self.target = None
        self.target_ns = None
        self.authority = None
        self.last_tick_ns = None
        self.absolute_localization_seen = False
        self.load_error = ""
        self.load_configuration()

        self.wrench_pub = self.create_publisher(WrenchStamped, "/control/pid/wrench", 10)
        self.candidate_pub = self.create_publisher(
            ThrusterCommand, "/control/candidates/pid", 10
        )
        self.status_pub = self.create_publisher(PidStatus, "/control/pid/status", 10)
        self.create_subscription(BodyState, "/robot/body_state", self.on_body, 20)
        self.create_subscription(
            TrajectoryTarget, "/runtime/trajectory_target", self.on_target, 20
        )
        self.create_subscription(
            ControlAuthorityStatus,
            "/control/authority/status",
            self.on_authority,
            20,
        )
        rate = max(1.0, float(self.get_parameter("control_rate_hz").value))
        self.timer = self.create_timer(1.0 / rate, self.tick)

    def load_configuration(self):
        try:
            pid_path = Path(str(self.get_parameter("pid_config_path").value))
            pid_data = yaml.safe_load(pid_path.read_text(encoding="utf-8")) or {}
            self.pid_configured = bool(pid_data.get("configured", False))
            self.outer_position_kp = vec(pid_data["outer_position_kp"], 3)
            self.outer_orientation_kp = vec(pid_data["outer_orientation_kp"], 3)
            self.max_linear_velocity = vec(pid_data["max_linear_velocity_mps"], 3)
            self.max_angular_velocity = vec(pid_data["max_angular_velocity_rps"], 3)
            kp = vec(pid_data["inner_kp"], 6)
            ki = vec(pid_data["inner_ki"], 6)
            kd = vec(pid_data["inner_kd"], 6)
            integral = vec(pid_data["integral_limit"], 6)
            output = vec(pid_data["wrench_limit"], 6)
            cutoff = float(pid_data.get("derivative_cutoff_hz", 5.0))
            self.pid = SixAxisPid(
                [
                    PidGains(
                        float(kp[i]),
                        float(ki[i]),
                        float(kd[i]),
                        float(integral[i]),
                        float(output[i]),
                        cutoff,
                    )
                    for i in range(6)
                ]
            )
            self.allocator = ThrusterAllocator.from_yaml(
                str(self.get_parameter("thruster_config_path").value)
            )
            canonical = json.dumps(pid_data, sort_keys=True, separators=(",", ":"))
            pid_hash = hashlib.sha256(canonical.encode("utf-8")).hexdigest()[:16]
            self.configuration_hash = f"{pid_hash}:{self.allocator.config_hash}"
        except Exception as exc:
            self.load_error = str(exc)
            self.pid_configured = False
            self.outer_position_kp = np.zeros(3)
            self.outer_orientation_kp = np.zeros(3)
            self.max_linear_velocity = np.zeros(3)
            self.max_angular_velocity = np.zeros(3)
            self.pid = SixAxisPid([PidGains(0, 0, 0, 0, 0) for _ in range(6)])
            self.allocator = None
            self.configuration_hash = ""
            self.get_logger().error(f"PID configuration rejected: {exc}")

    def on_body(self, message):
        self.body = message
        self.body_ns = self.get_clock().now().nanoseconds
        if message.state_valid:
            self.absolute_localization_seen = True

    def on_target(self, message):
        self.target = message
        self.target_ns = self.get_clock().now().nanoseconds

    def on_authority(self, message):
        self.authority = message
        if not message.armed or message.selected_source != "pid":
            self.pid.reset()

    def input_status(self, now_ns):
        missing = []
        maximum_age = float(self.get_parameter("max_input_age_s").value)
        body_age = math.inf if self.body_ns is None else (now_ns - self.body_ns) * 1e-9
        target_age = math.inf if self.target_ns is None else (now_ns - self.target_ns) * 1e-9
        if not self.pid_configured:
            missing.append("pid_config_not_confirmed")
        if self.allocator is None:
            missing.append("thruster_config_invalid")
        elif not self.allocator.measured:
            missing.append("thruster_geometry_not_measured")
        if self.body is None or body_age > maximum_age:
            missing.append("/robot/body_state")
        if self.target is None or target_age > maximum_age:
            missing.append("/runtime/trajectory_target")
        if self.body is not None:
            if not self.body.linear_velocity_valid:
                missing.append("linear_velocity_invalid")
            if not (self.body.state_valid or self.body.position_estimated):
                missing.append("localization_invalid")
            if self.body.position_estimated and not str(
                self.body.localization_source
            ).startswith("ZED VIO"):
                missing.append("estimated_pose_source_not_allowed")
            if not self.absolute_localization_seen:
                missing.append("absolute_localization_not_seen")
            try:
                quaternion_from_message(self.body.pose.orientation)
            except ValueError:
                missing.append("body_quaternion_invalid")
        if self.target is not None and not self.target.valid:
            missing.append("trajectory_target_invalid")
        return not missing, missing, body_age, target_age

    def tick(self):
        now = self.get_clock().now()
        now_ns = now.nanoseconds
        ready, missing, body_age, target_age = self.input_status(now_ns)
        if not ready:
            self.pid.reset()
            self.publish_disabled(now, missing, body_age, target_age)
            self.last_tick_ns = now_ns
            return
        dt = (
            1.0 / max(1.0, float(self.get_parameter("control_rate_hz").value))
            if self.last_tick_ns is None
            else (now_ns - self.last_tick_ns) * 1e-9
        )
        self.last_tick_ns = now_ns
        if not 1e-4 <= dt <= 0.1:
            self.pid.reset()
            self.publish_disabled(now, ["control_dt_invalid"], body_age, target_age)
            return

        body = self.body
        target = self.target
        current_q = quaternion_from_message(body.pose.orientation)
        target_q = quaternion_from_message(target.target_pose.orientation)
        world_to_body = quaternion_conjugate(current_q)
        current_position = vec(
            [body.pose.position.x, body.pose.position.y, body.pose.position.z], 3
        )
        target_position = vec(
            [
                target.target_pose.position.x,
                target.target_pose.position.y,
                target.target_pose.position.z,
            ],
            3,
        )
        position_error_body = quaternion_apply(
            world_to_body, target_position - current_position
        )
        orientation_error_body = quaternion_error_body(current_q, target_q)
        target_linear_body = quaternion_apply(
            world_to_body,
            [
                target.target_twist.linear.x,
                target.target_twist.linear.y,
                target.target_twist.linear.z,
            ],
        )
        target_angular_body = quaternion_apply(
            world_to_body,
            [
                target.target_twist.angular.x,
                target.target_twist.angular.y,
                target.target_twist.angular.z,
            ],
        )
        desired_linear = np.clip(
            target_linear_body + self.outer_position_kp * position_error_body,
            -self.max_linear_velocity,
            self.max_linear_velocity,
        )
        desired_angular = np.clip(
            target_angular_body + self.outer_orientation_kp * orientation_error_body,
            -self.max_angular_velocity,
            self.max_angular_velocity,
        )
        actual = vec(
            [
                body.twist.linear.x,
                body.twist.linear.y,
                body.twist.linear.z,
                body.twist.angular.x,
                body.twist.angular.y,
                body.twist.angular.z,
            ],
            6,
        )
        wrench = self.pid.step(np.concatenate([desired_linear, desired_angular]), actual, dt)
        allocation = self.allocator.allocate(wrench)

        wrench_message = WrenchStamped()
        wrench_message.header.stamp = now.to_msg()
        wrench_message.header.frame_id = "base_link"
        wrench_message.wrench.force.x, wrench_message.wrench.force.y, wrench_message.wrench.force.z = [
            float(value) for value in wrench[:3]
        ]
        (
            wrench_message.wrench.torque.x,
            wrench_message.wrench.torque.y,
            wrench_message.wrench.torque.z,
        ) = [float(value) for value in wrench[3:]]
        self.wrench_pub.publish(wrench_message)

        command = ThrusterCommand()
        command.header = wrench_message.header
        command.normalized = list(allocation.commands)
        command.enable = True
        command.source = "pid_controller"
        self.candidate_pub.publish(command)
        self.publish_status(
            now, True, True, [], body_age, target_age,
            allocation.residual, allocation.saturation_fraction, "ready"
        )

    def publish_disabled(self, now, missing, body_age, target_age):
        command = ThrusterCommand()
        command.header.stamp = now.to_msg()
        command.header.frame_id = "base_link"
        command.normalized = [0.0] * 8
        command.enable = False
        command.source = "pid_controller"
        self.candidate_pub.publish(command)
        self.publish_status(
            now, False, False, missing, body_age, target_age,
            math.inf, 0.0, self.load_error or ", ".join(missing)
        )

    def publish_status(
        self, now, ready, producing, missing, body_age, target_age,
        residual, saturation, message
    ):
        status = PidStatus()
        status.header.stamp = now.to_msg()
        status.ready = bool(ready)
        status.producing_command = bool(producing)
        status.missing_inputs = list(missing)
        status.body_state_age_s = float(body_age)
        status.target_age_s = float(target_age)
        status.allocation_rank = int(self.allocator.rank if self.allocator else 0)
        status.allocation_condition = float(
            self.allocator.condition if self.allocator else math.inf
        )
        status.allocation_residual = float(residual)
        status.saturation_fraction = float(saturation)
        status.configuration_hash = self.configuration_hash
        status.message = str(message)
        self.status_pub.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = SixDofPidNode()
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
