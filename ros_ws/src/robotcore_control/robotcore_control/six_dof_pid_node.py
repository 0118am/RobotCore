"""Direct altitude and station-hold actuator-domain PID controller."""

from __future__ import annotations

from collections import deque
import hashlib
import json
import math
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import WrenchStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
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
    altitude_collective_pwm_commands,
    altitude_level_pwm_commands,
    altitude_station_pwm_commands,
    altitude_velocity_setpoint,
    first_order_low_pass,
    level_attitude_pd_efforts,
    normalize_quaternion,
    quaternion_apply,
    quaternion_conjugate,
    quaternion_error_body,
    quaternion_slerp,
    reject_vector_outlier,
    timestamped_rate_prediction,
    vec,
)


PWM_HARDWARE_SPAN_US = 500.0
PWM_MODEL_LIMIT_US = 200.0


def quaternion_from_message(message):
    return normalize_quaternion([message.w, message.x, message.y, message.z])


class SixDofPidNode(Node):
    """Convert BodyState, external IMU, and trajectory targets to PID commands."""

    def __init__(self):
        super().__init__("six_dof_pid_controller")
        self.declare_parameter("control_rate_hz", 50.0)
        self.declare_parameter("max_input_age_s", 0.15)
        self.declare_parameter("pwm_limit_us", PWM_MODEL_LIMIT_US)
        self.declare_parameter("altitude_pwm_kp", 2.0)
        self.declare_parameter("altitude_pwm_ki", 0.8)
        self.declare_parameter("altitude_pwm_kd", 0.15)
        self.declare_parameter("altitude_pwm_integral_limit", 0.4)
        self.declare_parameter("altitude_velocity_filter_time_constant_s", 0.20)
        self.declare_parameter("altitude_pwm_command_sign", -1.0)
        self.declare_parameter("altitude_position_kp", 0.35)
        self.declare_parameter("station_surge_pwm_kp", 1.5)
        self.declare_parameter("station_sway_pwm_kp", 1.25)
        self.declare_parameter("station_yaw_pwm_kp", 0.55)
        self.declare_parameter("station_yaw_heading_kp", 0.60)
        self.declare_parameter("station_horizontal_axis_limit", 0.12)
        self.declare_parameter("station_yaw_axis_limit", 0.20)
        self.declare_parameter("station_surge_rate_limit", 0.40)
        self.declare_parameter("fast_station_surge_rate_limit", 0.50)
        self.declare_parameter("station_yaw_rate_limit", 0.60)
        self.declare_parameter("fast_station_level_pwm_limit", 0.10)
        self.declare_parameter("fast_station_roll_angle_kp", 0.45)
        self.declare_parameter("fast_station_pitch_angle_kp", 0.45)
        self.declare_parameter("fast_station_roll_rate_kp", 0.30)
        self.declare_parameter("fast_station_pitch_rate_kp", 0.55)
        self.declare_parameter("imu_topic", "/sensors/external_imu")
        self.declare_parameter("imu_rate_history_samples", 5)
        self.declare_parameter("imu_rate_outlier_limit_rps", 0.20)
        self.declare_parameter("imu_rate_prediction_horizon_s", 0.04)
        self.declare_parameter("imu_rate_prediction_accel_limit_rps2", 4.0)
        self.declare_parameter("imu_rate_prediction_delta_limit_rps", 0.12)
        self.declare_parameter("imu_rate_filter_time_constant_s", 0.01)
        self.declare_parameter("imu_orientation_filter_time_constant_s", 0.05)
        self.declare_parameter(
            "pid_config_path", "src/robotcore_control/config/real_pool_pid.yaml"
        )
        self.body = None
        self.body_ns = None
        self.imu_angular_velocity = None
        self.imu_orientation = None
        self.imu_ns = None
        self.imu_angular_velocity_filtered = None
        self.imu_orientation_filtered = None
        self.imu_filter_ns = None
        requested_history = int(
            self.get_parameter("imu_rate_history_samples").value
        )
        self.imu_rate_samples = deque(maxlen=max(3, min(requested_history, 15)))
        self.target = None
        self.target_ns = None
        self.last_tick_ns = None
        self.altitude_hold_active = False
        self.station_hold_active = False
        self.fast_station_hold_active = False
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
        self.command_pub = self.create_publisher(
            ThrusterCommand, "/control/pid/thruster_cmd", 10
        )
        self.status_pub = self.create_publisher(PidStatus, "/control/pid/status", 10)
        self.create_service(Trigger, "/control/pid/reload", self.on_reload)
        self.create_service(GetPidConfig, "/control/pid/config", self.on_get_config)
        self.create_subscription(BodyState, "/robot/body_state", self.on_body, 20)
        self.create_subscription(
            Imu,
            str(self.get_parameter("imu_topic").value),
            self.on_imu,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            TrajectoryTarget, "/runtime/trajectory_target", self.on_target, 20
        )
        self.create_subscription(
            ControlAuthorityStatus,
            "/control/authority/status",
            self.on_authority_status,
            20,
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
            self.inner_kp = vec(pid_data["inner_kp"], 6)
            cutoff = float(pid_data.get("derivative_cutoff_hz", 5.0))
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
            self.altitude_position_kp = float(
                self.get_parameter("altitude_position_kp").value
            )
            if (
                not math.isfinite(self.altitude_position_kp)
                or self.altitude_position_kp < 0.0
            ):
                raise ValueError("altitude position gain must be finite and non-negative")
            station_values = [
                float(self.get_parameter("station_surge_pwm_kp").value),
                float(self.get_parameter("station_sway_pwm_kp").value),
                float(self.get_parameter("station_yaw_pwm_kp").value),
                float(self.get_parameter("station_yaw_heading_kp").value),
                float(self.get_parameter("station_horizontal_axis_limit").value),
                float(self.get_parameter("station_yaw_axis_limit").value),
                float(self.get_parameter("station_surge_rate_limit").value),
                float(self.get_parameter("fast_station_surge_rate_limit").value),
                float(self.get_parameter("station_yaw_rate_limit").value),
                float(self.get_parameter("fast_station_level_pwm_limit").value),
            ]
            if not all(math.isfinite(value) for value in station_values):
                raise ValueError("station PWM gains and limit must be finite")
            if any(value < 0.0 for value in station_values):
                raise ValueError("station PWM gains and limit must be non-negative")
            (
                self.station_surge_pwm_kp,
                self.station_sway_pwm_kp,
                self.station_yaw_pwm_kp,
                self.station_yaw_heading_kp,
                self.station_horizontal_axis_limit,
                self.station_yaw_axis_limit,
                self.station_surge_rate_limit,
                self.fast_station_surge_rate_limit,
                self.station_yaw_rate_limit,
                self.fast_station_level_pwm_limit,
            ) = station_values
            level_gain_values = [
                float(self.get_parameter("fast_station_roll_angle_kp").value),
                float(self.get_parameter("fast_station_pitch_angle_kp").value),
                float(self.get_parameter("fast_station_roll_rate_kp").value),
                float(self.get_parameter("fast_station_pitch_rate_kp").value),
            ]
            if not all(
                math.isfinite(value) and value >= 0.0
                for value in level_gain_values
            ):
                raise ValueError("level PD gains must be finite and non-negative")
            self.fast_station_level_angle_kp = np.asarray(
                level_gain_values[:2], dtype=np.float64
            )
            self.fast_station_level_rate_kp = np.asarray(
                level_gain_values[2:], dtype=np.float64
            )
            imu_rate_history_samples = int(
                self.get_parameter("imu_rate_history_samples").value
            )
            imu_rate_values = [
                float(self.get_parameter("imu_rate_outlier_limit_rps").value),
                float(self.get_parameter("imu_rate_prediction_horizon_s").value),
                float(
                    self.get_parameter(
                        "imu_rate_prediction_accel_limit_rps2"
                    ).value
                ),
                float(
                    self.get_parameter(
                        "imu_rate_prediction_delta_limit_rps"
                    ).value
                ),
                float(self.get_parameter("imu_rate_filter_time_constant_s").value),
            ]
            if not 3 <= imu_rate_history_samples <= 15:
                raise ValueError("IMU rate history must contain 3 to 15 samples")
            if not all(
                math.isfinite(value) and value >= 0.0 for value in imu_rate_values
            ):
                raise ValueError(
                    "IMU rate estimator settings must be finite and non-negative"
                )
            (
                self.imu_rate_outlier_limit_rps,
                self.imu_rate_prediction_horizon_s,
                self.imu_rate_prediction_accel_limit_rps2,
                self.imu_rate_prediction_delta_limit_rps,
                self.imu_rate_filter_time_constant_s,
            ) = imu_rate_values
            if self.imu_rate_samples.maxlen != imu_rate_history_samples:
                self.imu_rate_samples = deque(maxlen=imu_rate_history_samples)
            self.imu_orientation_filter_time_constant_s = float(
                self.get_parameter("imu_orientation_filter_time_constant_s").value
            )
            if (
                not math.isfinite(self.imu_rate_filter_time_constant_s)
                or self.imu_rate_filter_time_constant_s < 0.0
                or not math.isfinite(self.imu_orientation_filter_time_constant_s)
                or self.imu_orientation_filter_time_constant_s < 0.0
            ):
                raise ValueError(
                    "IMU filter time constants must be finite and non-negative"
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
                    "direct_station": {
                        "surge_pwm_kp": self.station_surge_pwm_kp,
                        "sway_pwm_kp": self.station_sway_pwm_kp,
                        "yaw_pwm_kp": self.station_yaw_pwm_kp,
                        "yaw_heading_kp": self.station_yaw_heading_kp,
                        "horizontal_axis_limit": self.station_horizontal_axis_limit,
                        "yaw_axis_limit": self.station_yaw_axis_limit,
                        "surge_rate_limit": self.station_surge_rate_limit,
                        "fast_surge_rate_limit": (
                            self.fast_station_surge_rate_limit
                        ),
                        "yaw_rate_limit": self.station_yaw_rate_limit,
                        "fast_level_pwm_limit": self.fast_station_level_pwm_limit,
                        "fast_level_angle_kp": (
                            self.fast_station_level_angle_kp.tolist()
                        ),
                        "fast_level_rate_kp": (
                            self.fast_station_level_rate_kp.tolist()
                        ),
                    },
                    "external_imu_feedback": {
                        "imu_topic": str(self.get_parameter("imu_topic").value),
                        "rate_history_samples": self.imu_rate_samples.maxlen,
                        "rate_outlier_limit_rps": self.imu_rate_outlier_limit_rps,
                        "rate_prediction_horizon_s": (
                            self.imu_rate_prediction_horizon_s
                        ),
                        "rate_prediction_accel_limit_rps2": (
                            self.imu_rate_prediction_accel_limit_rps2
                        ),
                        "rate_prediction_delta_limit_rps": (
                            self.imu_rate_prediction_delta_limit_rps
                        ),
                        "rate_filter_time_constant_s": (
                            self.imu_rate_filter_time_constant_s
                        ),
                        "orientation_filter_time_constant_s": (
                            self.imu_orientation_filter_time_constant_s
                        ),
                    },
                },
                sort_keys=True,
                separators=(",", ":"),
            )
            pid_hash = hashlib.sha256(canonical.encode("utf-8")).hexdigest()[:16]
            self.configuration_hash = pid_hash
        except Exception as exc:
            self.pid_document = {}
            self.load_error = str(exc)
            self.pid_configured = False
            self.outer_position_kp = np.zeros(3)
            self.outer_orientation_kp = np.zeros(3)
            self.max_linear_velocity = np.zeros(3)
            self.max_angular_velocity = np.zeros(3)
            self.inner_kp = np.zeros(6)
            self.altitude_pid = ConditionalPid(PidGains(0, 0, 0, 0, 0))
            self.altitude_velocity_filter_time_constant_s = 0.20
            self.altitude_pwm_command_sign = -1.0
            self.altitude_position_kp = 0.0
            self.station_surge_pwm_kp = 0.0
            self.station_sway_pwm_kp = 0.0
            self.station_yaw_pwm_kp = 0.0
            self.station_yaw_heading_kp = 0.0
            self.station_horizontal_axis_limit = 0.0
            self.station_yaw_axis_limit = 0.0
            self.station_surge_rate_limit = 0.0
            self.fast_station_surge_rate_limit = 0.0
            self.station_yaw_rate_limit = 0.0
            self.fast_station_level_pwm_limit = 0.0
            self.fast_station_level_angle_kp = np.zeros(2)
            self.fast_station_level_rate_kp = np.zeros(2)
            self.imu_rate_outlier_limit_rps = 0.20
            self.imu_rate_prediction_horizon_s = 0.04
            self.imu_rate_prediction_accel_limit_rps2 = 4.0
            self.imu_rate_prediction_delta_limit_rps = 0.12
            self.imu_rate_filter_time_constant_s = 0.01
            self.imu_orientation_filter_time_constant_s = 0.05
            self.configuration_hash = ""
            self.get_logger().error(f"PID configuration rejected: {exc}")

    def on_reload(self, _request, response):
        """Reload the complete active PID document before a new arm cycle."""

        self.load_configuration()
        self.reset_controllers()
        self.reset_imu_conditioning()
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

    def on_imu(self, message):
        orientation = np.asarray(
            [
                message.orientation.w,
                message.orientation.x,
                message.orientation.y,
                message.orientation.z,
            ],
            dtype=np.float64,
        )
        angular_velocity = np.asarray(
            [
                message.angular_velocity.x,
                message.angular_velocity.y,
                message.angular_velocity.z,
            ],
            dtype=np.float64,
        )
        if (
            message.header.frame_id != "base_link"
            or message.orientation_covariance[0] < 0.0
            or not np.all(np.isfinite(angular_velocity))
            or not np.all(np.isfinite(orientation))
            or np.linalg.norm(orientation) < 1e-9
        ):
            return
        orientation = normalize_quaternion(orientation)
        now_ns = self.get_clock().now().nanoseconds
        sample_ns = (
            int(message.header.stamp.sec) * 1_000_000_000
            + int(message.header.stamp.nanosec)
        )
        dt = (
            math.inf
            if self.imu_filter_ns is None
            else (sample_ns - self.imu_filter_ns) * 1e-9
        )
        if not 0.0 < dt <= 0.2:
            self.imu_rate_samples.clear()
            self.imu_rate_samples.append((sample_ns, angular_velocity.copy()))
            self.imu_angular_velocity_filtered = angular_velocity.copy()
            self.imu_orientation_filtered = orientation.copy()
        else:
            accepted_rate = reject_vector_outlier(
                [sample[1] for sample in self.imu_rate_samples],
                angular_velocity,
                self.imu_rate_outlier_limit_rps,
            )
            self.imu_rate_samples.append((sample_ns, accepted_rate))
            predicted_rate = timestamped_rate_prediction(
                [sample[0] for sample in self.imu_rate_samples],
                [sample[1] for sample in self.imu_rate_samples],
                self.imu_rate_prediction_horizon_s,
                self.imu_rate_prediction_accel_limit_rps2,
                self.imu_rate_prediction_delta_limit_rps,
            )
            rate_alpha = (
                1.0
                if self.imu_rate_filter_time_constant_s <= 0.0
                else -math.expm1(-dt / self.imu_rate_filter_time_constant_s)
            )
            self.imu_angular_velocity_filtered += rate_alpha * (
                predicted_rate - self.imu_angular_velocity_filtered
            )
            orientation_alpha = (
                1.0
                if self.imu_orientation_filter_time_constant_s <= 0.0
                else -math.expm1(-dt / self.imu_orientation_filter_time_constant_s)
            )
            self.imu_orientation_filtered = quaternion_slerp(
                self.imu_orientation_filtered, orientation, orientation_alpha
            )
        self.imu_orientation = orientation
        self.imu_angular_velocity = angular_velocity
        self.imu_ns = now_ns
        self.imu_filter_ns = sample_ns

    def on_authority_status(self, message):
        """Follow the canonical PWM limit updated by the operator webpage."""

        command_limit = float(message.command_limit)
        maximum_limit = PWM_MODEL_LIMIT_US / PWM_HARDWARE_SPAN_US
        if not math.isfinite(command_limit) or not 0.0 <= command_limit <= maximum_limit:
            self.get_logger().warning(
                f"ignored invalid authority command limit {command_limit}"
            )
            return
        pwm_limit_us = command_limit * PWM_HARDWARE_SPAN_US
        if not math.isclose(pwm_limit_us, self.pwm_limit_us, abs_tol=1e-9):
            self.pwm_limit_us = pwm_limit_us
            self.reset_controllers()

    def on_target(self, message):
        self.target = message
        self.target_ns = self.get_clock().now().nanoseconds

    def reset_controllers(self):
        self.altitude_pid.reset()
        self.altitude_vertical_velocity_filtered = None

    def reset_imu_conditioning(self):
        self.imu_angular_velocity_filtered = None
        self.imu_orientation_filtered = None
        self.imu_filter_ns = None
        self.imu_rate_samples.clear()

    @property
    def command_limit(self):
        return self.pwm_limit_us / PWM_HARDWARE_SPAN_US

    def input_status(self, now_ns):
        missing = []
        maximum_age = float(self.get_parameter("max_input_age_s").value)
        body_age = math.inf if self.body_ns is None else (now_ns - self.body_ns) * 1e-9
        imu_age = math.inf if self.imu_ns is None else (now_ns - self.imu_ns) * 1e-9
        target_age = math.inf if self.target_ns is None else (now_ns - self.target_ns) * 1e-9
        target_mode = (
            str(self.target.trajectory_type).lower()
            if self.target is not None
            else ""
        )
        if not self.pid_configured:
            missing.append("pid_config_not_confirmed")
        if target_mode not in {
            "idle",
            "altitude_hold",
            "station_hold",
            "station_hold_fast",
            "spatial_figure_eight",
        }:
            missing.append("unsupported_trajectory_type")
        if self.body is None or body_age > maximum_age:
            missing.append("/robot/body_state")
        if (
            self.imu_orientation_filtered is None
            or self.imu_angular_velocity_filtered is None
            or imu_age > maximum_age
        ):
            missing.append("/sensors/external_imu")
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
                missing.append("localization_quaternion_invalid")
        if self.target is not None and not self.target.valid:
            missing.append("trajectory_target_invalid")
        return not missing, missing, body_age, imu_age, target_age

    def tick(self):
        now = self.get_clock().now()
        now_ns = now.nanoseconds
        ready, missing, body_age, imu_age, target_age = self.input_status(now_ns)
        if not ready:
            self.reset_controllers()
            self.publish_disabled(now, missing, body_age, imu_age, target_age)
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
            self.publish_disabled(
                now, ["control_dt_invalid"], body_age, imu_age, target_age
            )
            return

        body = self.body
        target = self.target
        # Localization attitude is used only to rotate map-frame position and
        # linear-velocity commands into base_link. It comes from the ZED/Tag
        # localization EKF and never enters the attitude feedback loop.
        localization_q = quaternion_from_message(body.pose.orientation)
        map_to_body = quaternion_conjugate(localization_q)

        # Attitude and angular-rate feedback come directly from the external
        # IMU. The IMU does not pass through the localization EKF.
        current_q = self.imu_orientation_filtered
        imu_reference_to_body = quaternion_conjugate(current_q)
        target_q = quaternion_from_message(target.target_pose.orientation)
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
            map_to_body, target_position - current_position
        )
        orientation_error_body = quaternion_error_body(current_q, target_q)
        target_linear_body = quaternion_apply(
            map_to_body,
            [
                target.target_twist.linear.x,
                target.target_twist.linear.y,
                target.target_twist.linear.z,
            ],
        )
        target_angular_body = quaternion_apply(
            imu_reference_to_body,
            [
                target.target_twist.angular.x,
                target.target_twist.angular.y,
                target.target_twist.angular.z,
            ],
        )
        actual_linear_body = vec(
            [
                body.twist.linear.x,
                body.twist.linear.y,
                body.twist.linear.z,
            ],
            3,
        )
        target_mode = str(target.trajectory_type).lower()
        idle_mode = target_mode == "idle"
        altitude_mode = target_mode == "altitude_hold"
        station_mode = target_mode in {
            "station_hold",
            "station_hold_fast",
            "spatial_figure_eight",
        }
        fast_station_mode = target_mode == "station_hold_fast"
        direct_altitude_mode = altitude_mode or station_mode
        if (
            direct_altitude_mode != self.altitude_hold_active
            or station_mode != self.station_hold_active
            or fast_station_mode != self.fast_station_hold_active
        ):
            self.reset_controllers()
        self.altitude_hold_active = direct_altitude_mode
        self.station_hold_active = station_mode
        self.fast_station_hold_active = fast_station_mode

        if idle_mode:
            # Select+Arm precedes the managed task's explicit Start. Keep a
            # fresh enabled PID command for the authority gate, but never run a
            # hold controller or move a thruster during that hand-off window.
            self.reset_controllers()
            commands = np.zeros(8, dtype=np.float64)
            wrench = np.zeros(6, dtype=np.float64)
            allocation_residual = 0.0
            allocation_saturation = 0.0
            status_message = "ready; neutral until Start"
        elif direct_altitude_mode:
            # All direct hold modes use the same map-Z actuator-domain PID.
            # Station modes add independent PWM-domain planar-position and
            # heading P loops on T5--T8. The fast station variant also closes
            # roll/pitch attitude-rate loops on vertical T1--T4.
            actual_linear_world = quaternion_apply(
                localization_q, actual_linear_body
            )
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
                self.altitude_position_kp,
            )
            level_efforts = np.zeros(2, dtype=np.float64)
            level_raw = np.zeros(2, dtype=np.float64)
            level_saturated = False
            altitude_output_limit = self.command_limit
            if fast_station_mode:
                target_level_rates = np.clip(
                    target_angular_body[:2],
                    -self.max_angular_velocity[:2],
                    self.max_angular_velocity[:2],
                )
                level_limit = min(
                    self.command_limit, self.fast_station_level_pwm_limit
                )
                level_efforts, level_raw, level_saturated = (
                    level_attitude_pd_efforts(
                        orientation_error_body[:2],
                        target_level_rates,
                        self.imu_angular_velocity_filtered[:2],
                        self.fast_station_level_angle_kp,
                        self.fast_station_level_rate_kp,
                        level_limit,
                    )
                )
                # Roll/pitch differential is attitude-priority. The altitude
                # integrator sees only the PWM headroom that remains, so it
                # cannot wind up behind a saturated individual T1--T4 channel.
                altitude_output_limit = max(
                    0.0,
                    self.command_limit - float(np.sum(np.abs(level_efforts))),
                )

            controller_effort = self.altitude_pid.step(
                setpoint=desired_vertical_velocity,
                measurement=self.altitude_vertical_velocity_filtered,
                dt=dt,
                output_limit=altitude_output_limit,
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
            if fast_station_mode:
                commands = altitude_level_pwm_commands(
                    controller_effort,
                    float(level_efforts[0]),
                    float(level_efforts[1]),
                    self.command_limit,
                    self.altitude_pwm_command_sign,
                )
            # Preserve the existing diagnostic topic while making its height
            # component explicitly the normalized direct PID effort rather
            # than a claimed force in newtons.
            wrench = np.zeros(6, dtype=np.float64)
            wrench[2] = controller_effort
            allocation_residual = 0.0
            allocation_saturation = float(self.altitude_pid.saturated)
            if station_mode:
                surge_rate_limit = (
                    self.fast_station_surge_rate_limit
                    if fast_station_mode
                    else self.station_surge_rate_limit
                )
                desired_surge_velocity = float(
                    np.clip(
                        target_linear_body[0]
                        + self.outer_position_kp[0] * position_error_body[0],
                        -surge_rate_limit,
                        surge_rate_limit,
                    )
                )
                desired_sway_velocity = float(
                    np.clip(
                        target_linear_body[1]
                        + self.outer_position_kp[1] * position_error_body[1],
                        -self.max_linear_velocity[1],
                        self.max_linear_velocity[1],
                    )
                )
                desired_yaw_rate = float(
                    np.clip(
                        target_angular_body[2]
                        + self.station_yaw_heading_kp * orientation_error_body[2],
                        -self.station_yaw_rate_limit,
                        self.station_yaw_rate_limit,
                    )
                )
                horizontal_axis_limit = (
                    self.command_limit
                    if fast_station_mode
                    else min(self.command_limit, self.station_horizontal_axis_limit)
                )
                yaw_axis_limit = min(
                    self.command_limit, self.station_yaw_axis_limit
                )
                surge_raw = self.station_surge_pwm_kp * (
                    desired_surge_velocity - float(actual_linear_body[0])
                )
                sway_raw = self.station_sway_pwm_kp * (
                    desired_sway_velocity - float(actual_linear_body[1])
                )
                yaw_raw = self.station_yaw_pwm_kp * (
                    desired_yaw_rate
                    - float(self.imu_angular_velocity_filtered[2])
                )
                surge_effort = float(
                    np.clip(surge_raw, -horizontal_axis_limit, horizontal_axis_limit)
                )
                sway_effort = float(
                    np.clip(sway_raw, -horizontal_axis_limit, horizontal_axis_limit)
                )
                yaw_effort = float(
                    np.clip(yaw_raw, -yaw_axis_limit, yaw_axis_limit)
                )
                commands = altitude_station_pwm_commands(
                    controller_effort,
                    surge_effort,
                    sway_effort,
                    yaw_effort,
                    self.command_limit,
                    self.altitude_pwm_command_sign,
                )
                if fast_station_mode:
                    commands[:4] = altitude_level_pwm_commands(
                        controller_effort,
                        float(level_efforts[0]),
                        float(level_efforts[1]),
                        self.command_limit,
                        self.altitude_pwm_command_sign,
                    )[:4]
                horizontal_saturated = (
                    abs(surge_raw) > horizontal_axis_limit
                    or abs(sway_raw) > horizontal_axis_limit
                    or abs(yaw_raw) > yaw_axis_limit
                    or (
                        self.command_limit > 0.0
                        and np.any(
                            np.abs(commands[4:])
                            >= self.command_limit - 1e-9
                        )
                    )
                )
                wrench[0] = surge_effort
                wrench[1] = sway_effort
                wrench[3] = float(level_efforts[0])
                wrench[4] = float(level_efforts[1])
                wrench[5] = yaw_effort
                allocation_saturation = float(
                    self.altitude_pid.saturated
                    or horizontal_saturated
                    or level_saturated
                )
                if fast_station_mode:
                    status_message = (
                        f"fast station height {target.target_pose.position.z:.3f} m; "
                        "roll/pitch level PD "
                        f"{level_efforts[0]:+.3f}/{level_efforts[1]:+.3f}; "
                        "surge/sway/yaw P "
                        f"{surge_effort:+.3f}/{sway_effort:+.3f}/{yaw_effort:+.3f}; "
                        f"individual PWM <= {self.pwm_limit_us:.0f} us"
                    )
                else:
                    status_message = (
                        f"station height {target.target_pose.position.z:.3f} m; "
                        "surge/sway/yaw P "
                        f"{surge_effort:+.3f}/{sway_effort:+.3f}/{yaw_effort:+.3f}; "
                        "balanced direct T1--T8 PWM"
                    )
            else:
                status_message = (
                    f"altitude hold {target.target_pose.position.z:.3f} m; "
                    f"PID {controller_effort:+.3f}, PWM {commands[0]:+.3f}"
                )
        else:
            self.publish_disabled(
                now,
                ["unsupported_trajectory_type"],
                body_age,
                imu_age,
                target_age,
            )
            return

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
        self.command_pub.publish(command)
        self.publish_status(
            now, True, True, [], body_age, imu_age, target_age,
            allocation_residual, allocation_saturation, status_message
        )

    def publish_disabled(self, now, missing, body_age, imu_age, target_age):
        command = ThrusterCommand()
        command.header.stamp = now.to_msg()
        command.header.frame_id = "base_link"
        command.normalized = [0.0] * 8
        command.enable = False
        command.source = "pid_controller"
        self.command_pub.publish(command)
        self.publish_status(
            now, False, False, missing, body_age, imu_age, target_age,
            math.inf, 0.0, self.load_error or ", ".join(missing)
        )

    def publish_status(
        self, now, ready, producing, missing, body_age, imu_age, target_age,
        residual, saturation, message
    ):
        status = PidStatus()
        status.header.stamp = now.to_msg()
        status.ready = bool(ready)
        status.producing_command = bool(producing)
        status.missing_inputs = list(missing)
        status.body_state_age_s = float(body_age)
        status.imu_age_s = float(imu_age)
        status.target_age_s = float(target_age)
        status.allocation_rank = 0
        status.allocation_condition = math.inf
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
