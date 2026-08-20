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
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64
from std_srvs.srv import Trigger
import yaml

from robotcore_interfaces.msg import (
    BodyState,
    ControlAuthorityStatus,
    PidStatus,
    ThrusterCommand,
    TrajectoryTarget,
)
from robotcore_interfaces.srv import GetPidConfig

from .control_math import (
    ConditionalPid,
    PidGains,
    SixAxisPid,
    altitude_collective_pwm_commands,
    altitude_velocity_setpoint,
    first_order_low_pass,
    manual_surge_yaw_commands,
    normalize_quaternion,
    quaternion_apply,
    quaternion_conjugate,
    quaternion_error_body,
    vec,
)
from .thruster_allocation import ThrusterAllocator


PWM_HARDWARE_SPAN_US = 500.0
PWM_MODEL_LIMIT_US = 200.0


def quaternion_from_message(message):
    return normalize_quaternion([message.w, message.x, message.y, message.z])


class SixDofPidNode(Node):
    """Convert full-pose trajectory targets to a PID command candidate."""

    def __init__(self):
        super().__init__("six_dof_pid_controller")
        self.declare_parameter("control_rate_hz", 30.0)
        self.declare_parameter("max_input_age_s", 0.15)
        self.declare_parameter("manual_input_age_s", 0.15)
        self.declare_parameter("pwm_limit_us", PWM_MODEL_LIMIT_US)
        self.declare_parameter("altitude_pwm_kp", 2.0)
        self.declare_parameter("altitude_pwm_ki", 0.6)
        self.declare_parameter("altitude_pwm_kd", 0.0)
        self.declare_parameter("altitude_pwm_integral_limit", 0.4)
        self.declare_parameter("altitude_velocity_filter_time_constant_s", 0.20)
        self.declare_parameter("altitude_pwm_command_sign", -1.0)
        self.declare_parameter("pid_config_path", "src/robotcore_control/config/real_pool_pid.yaml")
        self.declare_parameter(
            "thruster_config_path", "src/robotcore_control/config/real_pool_thrusters.yaml"
        )
        self.body = None
        self.body_ns = None
        self.target = None
        self.target_ns = None
        self.manual = None
        self.manual_ns = None
        self.authority = None
        self.last_tick_ns = None
        self.altitude_hold_active = False
        self.altitude_vertical_velocity_filtered = None
        self.absolute_localization_seen = False
        self.load_error = ""
        self.pwm_limit_us = float(
            np.clip(
                float(self.get_parameter("pwm_limit_us").value),
                0.0,
                PWM_MODEL_LIMIT_US,
            )
        )
        self.load_configuration()

        self.wrench_pub = self.create_publisher(WrenchStamped, "/control/pid/wrench", 10)
        self.candidate_pub = self.create_publisher(
            ThrusterCommand, "/control/candidates/pid", 10
        )
        self.status_pub = self.create_publisher(PidStatus, "/control/pid/status", 10)
        self.create_service(Trigger, "/control/pid/reload", self.on_reload)
        self.create_service(GetPidConfig, "/control/pid/config", self.on_get_config)
        self.create_subscription(BodyState, "/robot/body_state", self.on_body, 20)
        self.create_subscription(
            TrajectoryTarget, "/runtime/trajectory_target", self.on_target, 20
        )
        self.create_subscription(
            ThrusterCommand, "/control/candidates/manual", self.on_manual, 20
        )
        self.create_subscription(
            ControlAuthorityStatus,
            "/control/authority/status",
            self.on_authority,
            20,
        )
        pwm_limit_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            Float64, "/control/pwm_limit_us", self.on_pwm_limit, pwm_limit_qos
        )
        rate = max(1.0, float(self.get_parameter("control_rate_hz").value))
        self.timer = self.create_timer(1.0 / rate, self.tick)

    def load_configuration(self):
        self.load_error = ""
        try:
            pid_path = Path(str(self.get_parameter("pid_config_path").value))
            pid_data = yaml.safe_load(pid_path.read_text(encoding="utf-8")) or {}
            self.pid_document = pid_data
            self.pid_configured = bool(pid_data.get("configured", False))
            self.profile_name = str(pid_data.get("profile_name", pid_path.stem))
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
            altitude_values = [
                float(self.get_parameter("altitude_pwm_kp").value),
                float(self.get_parameter("altitude_pwm_ki").value),
                float(self.get_parameter("altitude_pwm_kd").value),
                float(self.get_parameter("altitude_pwm_integral_limit").value),
                float(
                    self.get_parameter(
                        "altitude_velocity_filter_time_constant_s"
                    ).value
                ),
                float(self.get_parameter("altitude_pwm_command_sign").value),
            ]
            if not all(math.isfinite(value) for value in altitude_values):
                raise ValueError("altitude PWM PID parameters must be finite")
            if any(value < 0.0 for value in altitude_values[:5]):
                raise ValueError("altitude PWM PID gains and filter must be non-negative")
            if math.isclose(altitude_values[5], 0.0, abs_tol=1e-12):
                raise ValueError("altitude PWM command sign must be non-zero")
            self.altitude_pid = ConditionalPid(
                PidGains(
                    kp=altitude_values[0],
                    ki=altitude_values[1],
                    kd=altitude_values[2],
                    integral_limit=altitude_values[3],
                    output_limit=1.0,
                    derivative_cutoff_hz=cutoff,
                )
            )
            self.altitude_velocity_filter_time_constant_s = altitude_values[4]
            self.altitude_pwm_command_sign = math.copysign(1.0, altitude_values[5])
            self.allocator = ThrusterAllocator.from_yaml(
                str(self.get_parameter("thruster_config_path").value)
            )
            canonical = json.dumps(
                {
                    "pid": pid_data,
                    "altitude_pwm": {
                        "kp": altitude_values[0],
                        "ki": altitude_values[1],
                        "kd": altitude_values[2],
                        "integral_limit": altitude_values[3],
                        "velocity_filter_time_constant_s": altitude_values[4],
                        "command_sign": self.altitude_pwm_command_sign,
                    },
                },
                sort_keys=True,
                separators=(",", ":"),
            )
            pid_hash = hashlib.sha256(canonical.encode("utf-8")).hexdigest()[:16]
            self.configuration_hash = f"{pid_hash}:{self.allocator.config_hash}"
        except Exception as exc:
            self.pid_document = {}
            self.load_error = str(exc)
            self.pid_configured = False
            self.outer_position_kp = np.zeros(3)
            self.outer_orientation_kp = np.zeros(3)
            self.max_linear_velocity = np.zeros(3)
            self.max_angular_velocity = np.zeros(3)
            self.pid = SixAxisPid([PidGains(0, 0, 0, 0, 0) for _ in range(6)])
            self.altitude_pid = ConditionalPid(PidGains(0, 0, 0, 0, 0))
            self.altitude_velocity_filter_time_constant_s = 0.20
            self.altitude_pwm_command_sign = -1.0
            self.allocator = None
            self.configuration_hash = ""
            self.get_logger().error(f"PID configuration rejected: {exc}")

    def on_reload(self, _request, response):
        """Reload the complete active PID document before a new arm cycle."""

        self.load_configuration()
        self.reset_controllers()
        self.last_tick_ns = None
        self.tick()
        response.success = not self.load_error
        response.message = (
            f"loaded PID profile {self.profile_name} ({self.configuration_hash})"
            if response.success
            else self.load_error
        )
        return response

    def on_get_config(self, _request, response):
        """Return the exact complete PID document currently used by this process."""

        response.success = not self.load_error
        response.config_json = json.dumps(
            self.pid_document, sort_keys=True, separators=(",", ":")
        )
        response.configuration_hash = self.configuration_hash
        response.message = (
            f"live PID profile {self.profile_name}"
            if response.success
            else self.load_error
        )
        return response

    def on_body(self, message):
        self.body = message
        self.body_ns = self.get_clock().now().nanoseconds
        if message.state_valid:
            self.absolute_localization_seen = True

    def on_target(self, message):
        self.target = message
        self.target_ns = self.get_clock().now().nanoseconds

    def on_manual(self, message):
        self.manual = message
        self.manual_ns = self.get_clock().now().nanoseconds

    def on_authority(self, message):
        self.authority = message
        if not message.armed or message.selected_source != "pid":
            self.reset_controllers()
            self.altitude_hold_active = False

    def on_pwm_limit(self, message):
        value = float(message.data)
        if not math.isfinite(value) or not 0.0 <= value <= PWM_MODEL_LIMIT_US:
            self.get_logger().warning(
                f"ignored invalid PWM limit {value}; expected 0..{PWM_MODEL_LIMIT_US:.0f} us"
            )
            return
        if not math.isclose(value, self.pwm_limit_us, abs_tol=1e-9):
            self.pwm_limit_us = value
            self.reset_controllers()

    def reset_controllers(self):
        self.pid.reset()
        self.altitude_pid.reset()
        self.altitude_vertical_velocity_filtered = None

    @property
    def command_limit(self):
        return self.pwm_limit_us / PWM_HARDWARE_SPAN_US

    def current_manual_surge_yaw(self, now_ns):
        maximum_age = float(self.get_parameter("manual_input_age_s").value)
        age = math.inf if self.manual_ns is None else (now_ns - self.manual_ns) * 1e-9
        if (
            self.manual is None
            or age > maximum_age
            or not self.manual.enable
            or self.manual.source != "web_operator"
        ):
            return np.zeros(8, dtype=np.float64)
        return manual_surge_yaw_commands(self.manual.normalized)

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
            self.reset_controllers()
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
            self.reset_controllers()
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
        target_mode = str(target.trajectory_type).lower()
        idle_mode = target_mode == "idle"
        altitude_mode = target_mode == "altitude_hold"
        if altitude_mode != self.altitude_hold_active:
            self.reset_controllers()
        self.altitude_hold_active = altitude_mode

        if idle_mode:
            # Select+Arm precedes the managed task's explicit Start. Keep a
            # fresh enabled candidate for the authority gate, but never run a
            # hold controller or move a thruster during that hand-off window.
            self.reset_controllers()
            commands = np.zeros(8, dtype=np.float64)
            wrench = np.zeros(6, dtype=np.float64)
            allocation_residual = 0.0
            allocation_saturation = 0.0
            status_message = "ready; neutral until Start"
        elif altitude_mode:
            # Altitude hold is a map-Z loop only.  Planar position and attitude
            # errors must not enter it while the operator drives surge/yaw.
            actual_linear_world = quaternion_apply(current_q, actual[:3])
            self.altitude_vertical_velocity_filtered = first_order_low_pass(
                self.altitude_vertical_velocity_filtered,
                float(actual_linear_world[2]),
                dt,
                self.altitude_velocity_filter_time_constant_s,
            )
            desired_vertical_velocity = altitude_velocity_setpoint(
                target_position[2],
                current_position[2],
                target.target_twist.linear.z,
                self.outer_position_kp[2],
            )
            controller_effort = self.altitude_pid.step(
                setpoint=desired_vertical_velocity,
                measurement=self.altitude_vertical_velocity_filtered,
                dt=dt,
                output_limit=self.command_limit,
            )
            # Height hold is a direct actuator-domain PID. No requested force
            # is inverted through the measured curves: PID effort is the one
            # common normalized PWM command for T1--T4, bounded only by the
            # live unified PWM Limit.
            commands = altitude_collective_pwm_commands(
                controller_effort,
                self.command_limit,
                self.altitude_pwm_command_sign,
            )
            collective = float(commands[0])
            manual = self.current_manual_surge_yaw(now_ns)
            # T1-T4 belong exclusively to automatic height control.  Only the
            # operator's forward/yaw projection is admitted on T5-T8.
            commands[4:] = manual[4:]
            commands[4:] = np.clip(
                commands[4:], -self.command_limit, self.command_limit
            )

            # Preserve the existing diagnostic topic while making its height
            # component explicitly the normalized direct PID effort rather
            # than a claimed force in newtons.
            wrench = np.zeros(6, dtype=np.float64)
            wrench[2] = controller_effort
            allocation_residual = 0.0
            allocation_saturation = float(self.altitude_pid.saturated)
            status_message = (
                f"altitude hold {target.target_pose.position.z:.3f} m; "
                f"PID {controller_effort:+.3f}, PWM {collective:+.3f}; "
                "manual surge/yaw"
            )
        else:
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
            wrench = self.pid.step(
                np.concatenate([desired_linear, desired_angular]), actual, dt
            )
            allocation = self.allocator.allocate(wrench)
            commands = np.clip(
                np.asarray(allocation.commands, dtype=np.float64),
                -self.command_limit,
                self.command_limit,
            )
            allocation_residual = allocation.residual
            allocation_saturation = allocation.saturation_fraction
            status_message = "ready"

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
        command.normalized = [float(value) for value in commands]
        command.enable = True
        command.source = "pid_controller"
        self.candidate_pub.publish(command)
        self.publish_status(
            now, True, True, [], body_age, target_age,
            allocation_residual, allocation_saturation, status_message
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
