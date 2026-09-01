"""Direct altitude and station-hold actuator-domain PID controller."""

from __future__ import annotations

from collections import deque
import hashlib
import json
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu

from robotcore_interfaces.msg import (
    BodyState,
    ControlAuthorityStatus,
    PidStatus,
    ThrusterCommand,
    TrajectoryTarget,
)
from .control_math import (
    ConditionalPid,
    PidGains,
    attitude_with_heading,
    altitude_collective_pwm_commands,
    altitude_station_pwm_commands,
    altitude_velocity_setpoint,
    conditional_axis_integral_effort,
    directional_velocity_feedforward,
    first_order_low_pass,
    integral_reset_after_error_crossing,
    level_attitude_rate_efforts,
    normalize_quaternion,
    quaternion_apply,
    quaternion_conjugate,
    quaternion_error_body,
    quaternion_slerp,
    reject_vector_outlier,
    slew_rate_limit,
    station_velocity_setpoints,
    station_horizontal_pwm_mix,
    surge_pitch_decoupling_effort,
    timestamped_rate_prediction,
    vec,
)


ACTION_PWM_SPAN_US = 250.0
# Existing PID gains are calibrated in an actuator-effort domain where 1.0
# represents 500 us. Convert that controller-owned effort once when publishing
# the shared direct action; RL actions never pass through this conversion.
PID_EFFORT_PWM_SPAN_US = 500.0
MAXIMUM_PWM_LIMIT_US = 250.0
FAST_STATION_SURGE_DIRECTION_DEADBAND = 0.01
FAST_STATION_LATERAL_ACTIVE_VELOCITY_MPS = 0.03
STATION_YAW_MANEUVER_RATE_THRESHOLD_RPS = 0.01
STATION_YAW_ERROR_CROSSING_HYSTERESIS_RAD = math.radians(0.2)


def quaternion_from_message(message):
    return normalize_quaternion([message.w, message.x, message.y, message.z])


class SixDofPidNode(Node):
    """Convert BodyState, external IMU, and trajectory targets to PID commands."""

    def __init__(self):
        super().__init__("six_dof_pid_controller")
        self.sensor_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.BEST_EFFORT
        )
        self.declare_parameter("control_rate_hz", 50.0)
        self.declare_parameter("max_input_age_s", 0.15)
        self.declare_parameter("pwm_limit_us", MAXIMUM_PWM_LIMIT_US)
        self.declare_parameter("altitude_pwm_kp", 1.4)
        self.declare_parameter("altitude_pwm_ki", 0.45)
        self.declare_parameter("altitude_pwm_kd", 0.22)
        self.declare_parameter("altitude_pwm_integral_limit", 0.5)
        self.declare_parameter("altitude_velocity_filter_time_constant_s", 0.10)
        self.declare_parameter("altitude_pwm_command_sign", -1.0)
        self.declare_parameter("altitude_position_kp", 0.25)
        self.declare_parameter("station_surge_pwm_kp", 1.5)
        self.declare_parameter("station_surge_pwm_ki", 0.25)
        self.declare_parameter("station_surge_pwm_kd", 0.0)
        self.declare_parameter("station_surge_integral_limit", 1.0)
        self.declare_parameter("station_sway_pwm_kp", 1.0)
        self.declare_parameter("fast_station_sway_pwm_kp", 0.60)
        self.declare_parameter(
            "fast_station_sway_pwm_feedforward_positive_gain", 1.80
        )
        self.declare_parameter(
            "fast_station_sway_pwm_feedforward_negative_gain", 1.90
        )
        self.declare_parameter("fast_station_sway_effort_slew_rate_per_s", 1.20)
        self.declare_parameter("station_sway_pwm_ki", 0.0)
        self.declare_parameter("station_sway_pwm_kd", 0.0)
        self.declare_parameter("station_sway_integral_limit", 0.5)
        self.declare_parameter("station_yaw_pwm_kp", 0.70)
        self.declare_parameter("station_yaw_pwm_ki", 0.12)
        self.declare_parameter("station_yaw_integral_limit", 0.8)
        self.declare_parameter("station_yaw_heading_kp", 0.90)
        self.declare_parameter("station_horizontal_axis_limit", 0.12)
        self.declare_parameter("station_yaw_axis_limit", 0.20)
        self.declare_parameter("station_surge_rate_limit", 0.40)
        self.declare_parameter("fast_station_surge_rate_limit", 0.50)
        self.declare_parameter("fast_station_sway_rate_limit", 0.20)
        self.declare_parameter("fast_station_lateral_yaw_axis_limit", 0.10)
        self.declare_parameter(
            "fast_station_sway_yaw_compensation_gain", 0.055
        )
        self.declare_parameter(
            "fast_station_lateral_yaw_effort_slew_rate_per_s", 0.30
        )
        self.declare_parameter("station_yaw_rate_limit", 0.60)
        self.declare_parameter("fast_station_level_pwm_limit", 0.10)
        self.declare_parameter("fast_station_roll_angle_to_rate_kp", 1.30)
        self.declare_parameter("fast_station_pitch_angle_to_rate_kp", 0.80)
        self.declare_parameter("fast_station_roll_rate_kp", 0.20)
        self.declare_parameter("fast_station_pitch_rate_kp", 0.35)
        self.declare_parameter("fast_station_pitch_rate_ki", 0.20)
        self.declare_parameter(
            "fast_station_pitch_rate_integral_effort_limit", 0.10
        )
        self.declare_parameter("fast_station_surge_pitch_decoupling_enabled", True)
        self.declare_parameter("fast_station_surge_pitch_forward_gain", 0.08)
        self.declare_parameter("fast_station_surge_pitch_reverse_gain", 0.06)
        self.declare_parameter("fast_station_surge_pitch_limit", 0.06)
        self.declare_parameter("imu_topic", "/sensors/external_imu")
        self.declare_parameter("imu_rate_history_samples", 5)
        self.declare_parameter("imu_rate_outlier_limit_rps", 0.20)
        self.declare_parameter("imu_rate_prediction_horizon_s", 0.04)
        self.declare_parameter("imu_rate_prediction_accel_limit_rps2", 4.0)
        self.declare_parameter("imu_rate_prediction_delta_limit_rps", 0.12)
        self.declare_parameter("imu_rate_filter_time_constant_s", 0.01)
        self.declare_parameter("imu_orientation_filter_time_constant_s", 0.03)
        # The controller is fail-closed unless the deployment YAML explicitly
        # approves the complete ROS-parameter set below.
        self.declare_parameter("configuration_ready", False)
        self.declare_parameter("station_position_kp", [0.0, 0.0])
        self.declare_parameter("station_sway_rate_limit", 0.0)
        self.declare_parameter("fast_station_level_rate_limit_rps", [0.0, 0.0])
        self.declare_parameter("derivative_cutoff_hz", 1.5)
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
        self.pid_selected = False
        self.altitude_hold_active = False
        self.station_hold_active = False
        self.fast_station_hold_active = False
        self.station_trajectory_phase = ""
        self.station_yaw_maneuver_active = False
        self.station_yaw_heading_error_sign = 0
        self.fast_station_sway_limited_effort = 0.0
        self.fast_station_lateral_yaw_limited_effort = 0.0
        self.fast_station_pitch_rate_integral_effort = 0.0
        self.fast_station_surge_direction = 0
        self.altitude_vertical_velocity_filtered = None
        self.absolute_localization_seen = False
        self.load_error = ""
        self.pwm_limit_us = float(
            np.clip(
                float(self.get_parameter("pwm_limit_us").value),
                0.0,
                MAXIMUM_PWM_LIMIT_US,
            )
        )
        self.load_configuration()

        self.command_pub = self.create_publisher(
            ThrusterCommand, "/control/pid/thruster_cmd", 1
        )
        self.status_pub = self.create_publisher(PidStatus, "/control/pid/status", 1)
        # The PID controller is mutually exclusive with RL/manual authority.
        # High-rate inputs are connected only while PID is selected so an idle
        # controller does not deserialize and filter every IMU/BodyState sample.
        self.body_sub = None
        self.imu_sub = None
        self.target_sub = None
        self.create_subscription(
            ControlAuthorityStatus,
            "/control/authority/status",
            self.on_authority_status,
            1,
        )
        rate = max(1.0, float(self.get_parameter("control_rate_hz").value))
        self.timer = self.create_timer(1.0 / rate, self.tick)
        self.timer.cancel()

    def load_configuration(self):
        """Validate and materialize the startup ROS parameter contract."""

        self.load_error = ""
        try:
            self.pid_configured = bool(
                self.get_parameter("configuration_ready").value
            )
            self.station_position_kp = vec(
                self.get_parameter("station_position_kp").value, 2
            )
            self.station_sway_rate_limit = float(
                self.get_parameter("station_sway_rate_limit").value
            )
            self.fast_station_level_rate_limit_rps = vec(
                self.get_parameter("fast_station_level_rate_limit_rps").value,
                2,
            )
            cutoff = float(self.get_parameter("derivative_cutoff_hz").value)
            if (
                np.any(self.station_position_kp < 0.0)
                or not math.isfinite(self.station_sway_rate_limit)
                or self.station_sway_rate_limit < 0.0
                or np.any(self.fast_station_level_rate_limit_rps < 0.0)
                or not math.isfinite(cutoff)
                or cutoff < 0.0
            ):
                raise ValueError(
                    "station gains, rate limits, and derivative cutoff must be "
                    "finite and non-negative"
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
                float(self.get_parameter("station_surge_pwm_ki").value),
                float(self.get_parameter("station_surge_pwm_kd").value),
                float(self.get_parameter("station_surge_integral_limit").value),
                float(self.get_parameter("station_sway_pwm_kp").value),
                float(self.get_parameter("fast_station_sway_pwm_kp").value),
                float(
                    self.get_parameter(
                        "fast_station_sway_pwm_feedforward_positive_gain"
                    ).value
                ),
                float(
                    self.get_parameter(
                        "fast_station_sway_pwm_feedforward_negative_gain"
                    ).value
                ),
                float(
                    self.get_parameter(
                        "fast_station_sway_effort_slew_rate_per_s"
                    ).value
                ),
                float(self.get_parameter("station_sway_pwm_ki").value),
                float(self.get_parameter("station_sway_pwm_kd").value),
                float(self.get_parameter("station_sway_integral_limit").value),
                float(self.get_parameter("station_yaw_pwm_kp").value),
                float(self.get_parameter("station_yaw_pwm_ki").value),
                float(self.get_parameter("station_yaw_integral_limit").value),
                float(self.get_parameter("station_yaw_heading_kp").value),
                float(self.get_parameter("station_horizontal_axis_limit").value),
                float(self.get_parameter("station_yaw_axis_limit").value),
                float(self.get_parameter("station_surge_rate_limit").value),
                float(self.get_parameter("fast_station_surge_rate_limit").value),
                float(self.get_parameter("fast_station_sway_rate_limit").value),
                float(
                    self.get_parameter("fast_station_lateral_yaw_axis_limit").value
                ),
                float(
                    self.get_parameter(
                        "fast_station_sway_yaw_compensation_gain"
                    ).value
                ),
                float(
                    self.get_parameter(
                        "fast_station_lateral_yaw_effort_slew_rate_per_s"
                    ).value
                ),
                float(self.get_parameter("station_yaw_rate_limit").value),
                float(self.get_parameter("fast_station_level_pwm_limit").value),
            ]
            if not all(math.isfinite(value) for value in station_values):
                raise ValueError("station PWM gains and limit must be finite")
            if any(value < 0.0 for value in station_values):
                raise ValueError("station PWM gains and limit must be non-negative")
            (
                self.station_surge_pwm_kp,
                self.station_surge_pwm_ki,
                self.station_surge_pwm_kd,
                self.station_surge_integral_limit,
                self.station_sway_pwm_kp,
                self.fast_station_sway_pwm_kp,
                self.fast_station_sway_pwm_feedforward_positive_gain,
                self.fast_station_sway_pwm_feedforward_negative_gain,
                self.fast_station_sway_effort_slew_rate_per_s,
                self.station_sway_pwm_ki,
                self.station_sway_pwm_kd,
                self.station_sway_integral_limit,
                self.station_yaw_pwm_kp,
                self.station_yaw_pwm_ki,
                self.station_yaw_integral_limit,
                self.station_yaw_heading_kp,
                self.station_horizontal_axis_limit,
                self.station_yaw_axis_limit,
                self.station_surge_rate_limit,
                self.fast_station_surge_rate_limit,
                self.fast_station_sway_rate_limit,
                self.fast_station_lateral_yaw_axis_limit,
                self.fast_station_sway_yaw_compensation_gain,
                self.fast_station_lateral_yaw_effort_slew_rate_per_s,
                self.station_yaw_rate_limit,
                self.fast_station_level_pwm_limit,
            ) = station_values
            self.station_surge_pid = ConditionalPid(
                PidGains(
                    kp=self.station_surge_pwm_kp,
                    ki=self.station_surge_pwm_ki,
                    kd=self.station_surge_pwm_kd,
                    integral_limit=self.station_surge_integral_limit,
                    output_limit=1.0,
                    derivative_cutoff_hz=cutoff,
                )
            )
            self.station_sway_pid = ConditionalPid(
                PidGains(
                    kp=self.station_sway_pwm_kp,
                    ki=self.station_sway_pwm_ki,
                    kd=self.station_sway_pwm_kd,
                    integral_limit=self.station_sway_integral_limit,
                    output_limit=1.0,
                    derivative_cutoff_hz=cutoff,
                )
            )
            self.fast_station_sway_pid = ConditionalPid(
                PidGains(
                    kp=self.fast_station_sway_pwm_kp,
                    ki=self.station_sway_pwm_ki,
                    kd=self.station_sway_pwm_kd,
                    integral_limit=self.station_sway_integral_limit,
                    output_limit=1.0,
                    derivative_cutoff_hz=cutoff,
                )
            )
            self.station_yaw_pid = ConditionalPid(
                PidGains(
                    kp=self.station_yaw_pwm_kp,
                    ki=self.station_yaw_pwm_ki,
                    kd=0.0,
                    integral_limit=self.station_yaw_integral_limit,
                    output_limit=1.0,
                    derivative_cutoff_hz=cutoff,
                )
            )
            level_gain_values = [
                float(
                    self.get_parameter(
                        "fast_station_roll_angle_to_rate_kp"
                    ).value
                ),
                float(
                    self.get_parameter(
                        "fast_station_pitch_angle_to_rate_kp"
                    ).value
                ),
                float(self.get_parameter("fast_station_roll_rate_kp").value),
                float(self.get_parameter("fast_station_pitch_rate_kp").value),
                float(self.get_parameter("fast_station_pitch_rate_ki").value),
                float(
                    self.get_parameter(
                        "fast_station_pitch_rate_integral_effort_limit"
                    ).value
                ),
                float(
                    self.get_parameter(
                        "fast_station_surge_pitch_forward_gain"
                    ).value
                ),
                float(
                    self.get_parameter(
                        "fast_station_surge_pitch_reverse_gain"
                    ).value
                ),
                float(
                    self.get_parameter("fast_station_surge_pitch_limit").value
                ),
            ]
            if not all(
                math.isfinite(value) and value >= 0.0
                for value in level_gain_values
            ):
                raise ValueError(
                    "level cascade and decoupling gains must be finite and "
                    "non-negative"
                )
            self.fast_station_level_angle_to_rate_kp = np.asarray(
                level_gain_values[:2], dtype=np.float64
            )
            self.fast_station_level_rate_kp = np.asarray(
                level_gain_values[2:4], dtype=np.float64
            )
            self.fast_station_pitch_rate_ki = level_gain_values[4]
            self.fast_station_pitch_rate_integral_effort_limit = (
                level_gain_values[5]
            )
            self.fast_station_surge_pitch_decoupling_enabled = bool(
                self.get_parameter(
                    "fast_station_surge_pitch_decoupling_enabled"
                ).value
            )
            self.fast_station_surge_pitch_forward_gain = level_gain_values[6]
            self.fast_station_surge_pitch_reverse_gain = level_gain_values[7]
            self.fast_station_surge_pitch_limit = level_gain_values[8]
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
                    "configuration_ready": self.pid_configured,
                    "derivative_cutoff_hz": cutoff,
                    "altitude_pwm": {
                        "position_kp": self.altitude_position_kp,
                        "kp": altitude_values[0],
                        "ki": altitude_values[1],
                        "kd": altitude_values[2],
                        "integral_limit": altitude_values[3],
                        "velocity_filter_time_constant_s": altitude_values[4],
                        "command_sign": self.altitude_pwm_command_sign,
                    },
                    "direct_station": {
                        "position_kp": self.station_position_kp.tolist(),
                        "sway_rate_limit": self.station_sway_rate_limit,
                        "fast_level_rate_limit_rps": (
                            self.fast_station_level_rate_limit_rps.tolist()
                        ),
                        "surge_velocity_pid": {
                            "kp": self.station_surge_pwm_kp,
                            "ki": self.station_surge_pwm_ki,
                            "kd": self.station_surge_pwm_kd,
                            "integral_limit": self.station_surge_integral_limit,
                        },
                        "sway_velocity_pid": {
                            "kp": self.station_sway_pwm_kp,
                            "ki": self.station_sway_pwm_ki,
                            "kd": self.station_sway_pwm_kd,
                            "integral_limit": self.station_sway_integral_limit,
                        },
                        "fast_sway_velocity_kp": (
                            self.fast_station_sway_pwm_kp
                        ),
                        "fast_sway_velocity_feedforward": {
                            "positive_gain": (
                                self.fast_station_sway_pwm_feedforward_positive_gain
                            ),
                            "negative_gain": (
                                self.fast_station_sway_pwm_feedforward_negative_gain
                            ),
                            "effort_slew_rate_per_s": (
                                self.fast_station_sway_effort_slew_rate_per_s
                            ),
                        },
                        "yaw_rate_pid": {
                            "kp": self.station_yaw_pwm_kp,
                            "ki": self.station_yaw_pwm_ki,
                            "kd": 0.0,
                            "integral_limit": self.station_yaw_integral_limit,
                        },
                        "yaw_heading_kp": self.station_yaw_heading_kp,
                        "horizontal_axis_limit": self.station_horizontal_axis_limit,
                        "yaw_axis_limit": self.station_yaw_axis_limit,
                        "surge_rate_limit": self.station_surge_rate_limit,
                        "fast_surge_rate_limit": (
                            self.fast_station_surge_rate_limit
                        ),
                        "fast_sway_rate_limit": (
                            self.fast_station_sway_rate_limit
                        ),
                        "fast_lateral_yaw_axis_limit": (
                            self.fast_station_lateral_yaw_axis_limit
                        ),
                        "fast_lateral_yaw_decoupling": {
                            "sway_compensation_gain": (
                                self.fast_station_sway_yaw_compensation_gain
                            ),
                            "effort_slew_rate_per_s": (
                                self.fast_station_lateral_yaw_effort_slew_rate_per_s
                            ),
                        },
                        "yaw_rate_limit": self.station_yaw_rate_limit,
                        "fast_level_pwm_limit": self.fast_station_level_pwm_limit,
                        "fast_level_angle_to_rate_kp": (
                            self.fast_station_level_angle_to_rate_kp.tolist()
                        ),
                        "fast_level_rate_kp": (
                            self.fast_station_level_rate_kp.tolist()
                        ),
                        "fast_pitch_rate_ki": (
                            self.fast_station_pitch_rate_ki
                        ),
                        "fast_pitch_rate_integral_effort_limit": (
                            self.fast_station_pitch_rate_integral_effort_limit
                        ),
                        "fast_surge_pitch_decoupling": {
                            "enabled": (
                                self.fast_station_surge_pitch_decoupling_enabled
                            ),
                            "forward_gain": (
                                self.fast_station_surge_pitch_forward_gain
                            ),
                            "reverse_gain": (
                                self.fast_station_surge_pitch_reverse_gain
                            ),
                            "limit": self.fast_station_surge_pitch_limit,
                        },
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
            self.load_error = str(exc)
            self.pid_configured = False
            self.station_position_kp = np.zeros(2)
            self.station_sway_rate_limit = 0.0
            self.fast_station_level_rate_limit_rps = np.zeros(2)
            self.altitude_pid = ConditionalPid(PidGains(0, 0, 0, 0, 0))
            self.altitude_velocity_filter_time_constant_s = 0.20
            self.altitude_pwm_command_sign = -1.0
            self.altitude_position_kp = 0.0
            self.station_surge_pwm_kp = 0.0
            self.station_surge_pwm_ki = 0.0
            self.station_surge_pwm_kd = 0.0
            self.station_surge_integral_limit = 0.0
            self.station_sway_pwm_kp = 0.0
            self.fast_station_sway_pwm_kp = 0.0
            self.fast_station_sway_pwm_feedforward_positive_gain = 0.0
            self.fast_station_sway_pwm_feedforward_negative_gain = 0.0
            self.fast_station_sway_effort_slew_rate_per_s = 0.0
            self.station_sway_pwm_ki = 0.0
            self.station_sway_pwm_kd = 0.0
            self.station_sway_integral_limit = 0.0
            self.station_surge_pid = ConditionalPid(PidGains(0, 0, 0, 0, 0))
            self.station_sway_pid = ConditionalPid(PidGains(0, 0, 0, 0, 0))
            self.fast_station_sway_pid = ConditionalPid(
                PidGains(0, 0, 0, 0, 0)
            )
            self.station_yaw_pwm_kp = 0.0
            self.station_yaw_pwm_ki = 0.0
            self.station_yaw_integral_limit = 0.0
            self.station_yaw_pid = ConditionalPid(PidGains(0, 0, 0, 0, 0))
            self.station_yaw_heading_kp = 0.0
            self.station_horizontal_axis_limit = 0.0
            self.station_yaw_axis_limit = 0.0
            self.station_surge_rate_limit = 0.0
            self.fast_station_surge_rate_limit = 0.0
            self.fast_station_sway_rate_limit = 0.0
            self.fast_station_lateral_yaw_axis_limit = 0.0
            self.fast_station_sway_yaw_compensation_gain = 0.0
            self.fast_station_lateral_yaw_effort_slew_rate_per_s = 0.0
            self.station_yaw_rate_limit = 0.0
            self.fast_station_level_pwm_limit = 0.0
            self.fast_station_level_angle_to_rate_kp = np.zeros(2)
            self.fast_station_level_rate_kp = np.zeros(2)
            self.fast_station_pitch_rate_ki = 0.0
            self.fast_station_pitch_rate_integral_effort_limit = 0.0
            self.fast_station_surge_pitch_decoupling_enabled = False
            self.fast_station_surge_pitch_forward_gain = 0.0
            self.fast_station_surge_pitch_reverse_gain = 0.0
            self.fast_station_surge_pitch_limit = 0.0
            self.imu_rate_outlier_limit_rps = 0.20
            self.imu_rate_prediction_horizon_s = 0.04
            self.imu_rate_prediction_accel_limit_rps2 = 4.0
            self.imu_rate_prediction_delta_limit_rps = 0.12
            self.imu_rate_filter_time_constant_s = 0.01
            self.imu_orientation_filter_time_constant_s = 0.03
            self.configuration_hash = ""
            self.get_logger().error(f"PID configuration rejected: {exc}")

    def on_body(self, message):
        self.body = message
        self.body_ns = self.get_clock().now().nanoseconds
        if message.state_valid:
            self.absolute_localization_seen = True

    def on_imu(self, message):
        if not self.pid_selected:
            return
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
        if self.body is None or not (
            self.body.state_valid or self.body.position_estimated
        ):
            return
        try:
            orientation = attitude_with_heading(
                orientation,
                quaternion_from_message(self.body.pose.orientation),
            )
        except ValueError:
            return
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
        """Run PID inputs/timer only while canonical authority selects PID."""

        selected = str(message.selected_source).strip().lower() == "pid"
        if selected != self.pid_selected:
            self.pid_selected = selected
            self.reset_controllers()
            self.last_tick_ns = None
            if selected:
                self._connect_pid_inputs()
                self.timer.reset()
            else:
                self.timer.cancel()
                self._disconnect_pid_inputs()

        action_limit = float(message.action_limit)
        if not math.isfinite(action_limit) or not 0.0 <= action_limit <= 1.0:
            self.get_logger().warning(
                f"ignored invalid authority action limit {action_limit}"
            )
            return
        pwm_limit_us = action_limit * ACTION_PWM_SPAN_US
        if not math.isclose(pwm_limit_us, self.pwm_limit_us, abs_tol=1e-9):
            self.pwm_limit_us = pwm_limit_us
            self.reset_controllers()

    def _connect_pid_inputs(self):
        if self.body_sub is None:
            self.body_sub = self.create_subscription(
                BodyState, "/robot/body_state", self.on_body, 1
            )
        if self.imu_sub is None:
            self.imu_sub = self.create_subscription(
                Imu,
                str(self.get_parameter("imu_topic").value),
                self.on_imu,
                self.sensor_qos,
            )
        if self.target_sub is None:
            self.target_sub = self.create_subscription(
                TrajectoryTarget, "/runtime/trajectory_target", self.on_target, 1
            )

    def _disconnect_pid_inputs(self):
        for attribute in ("body_sub", "imu_sub", "target_sub"):
            subscription = getattr(self, attribute)
            if subscription is not None:
                self.destroy_subscription(subscription)
                setattr(self, attribute, None)
        self.body = None
        self.body_ns = None
        self.imu_angular_velocity = None
        self.imu_orientation = None
        self.imu_ns = None
        self.imu_angular_velocity_filtered = None
        self.imu_orientation_filtered = None
        self.imu_filter_ns = None
        self.imu_rate_samples.clear()
        self.target = None
        self.target_ns = None

    def on_target(self, message):
        self.target = message
        self.target_ns = self.get_clock().now().nanoseconds

    def reset_controllers(self):
        self.altitude_pid.reset()
        self.station_surge_pid.reset()
        self.station_sway_pid.reset()
        self.fast_station_sway_pid.reset()
        self.station_yaw_pid.reset()
        self.station_yaw_maneuver_active = False
        self.station_yaw_heading_error_sign = 0
        self.fast_station_sway_limited_effort = 0.0
        self.fast_station_lateral_yaw_limited_effort = 0.0
        self.fast_station_pitch_rate_integral_effort = 0.0
        self.fast_station_surge_direction = 0
        self.altitude_vertical_velocity_filtered = None

    @property
    def command_limit(self):
        return self.pwm_limit_us / PID_EFFORT_PWM_SPAN_US

    def input_status(self, now_ns):
        missing = []
        maximum_age = float(self.get_parameter("max_input_age_s").value)
        body_age = math.inf if self.body_ns is None else (now_ns - self.body_ns) * 1e-9
        imu_age = math.inf if self.imu_ns is None else (now_ns - self.imu_ns) * 1e-9
        target_age = math.inf if self.target_ns is None else (now_ns - self.target_ns) * 1e-9
        target_trajectory_type = (
            str(self.target.trajectory_type).lower()
            if self.target is not None
            else ""
        )
        target_control_mode = (
            str(self.target.control_mode).lower()
            if self.target is not None
            else ""
        )
        if not self.pid_configured:
            missing.append("pid_parameters_not_approved")
        if target_trajectory_type not in {
            "idle",
            "hold",
            "spatial_lissajous",
            "circle",
            "racetrack",
            "straight_line",
        }:
            missing.append("unsupported_trajectory_type")
        if target_control_mode not in {
            "idle",
            "altitude_hold",
            "station_hold",
            "station_hold_fast",
        }:
            missing.append("unsupported_control_mode")
        compatible_target = (
            (target_trajectory_type == "idle" and target_control_mode == "idle")
            or (
                target_trajectory_type == "hold"
                and target_control_mode
                in {"altitude_hold", "station_hold", "station_hold_fast"}
            )
            or (
                target_trajectory_type
                in {"spatial_lissajous", "circle", "racetrack", "straight_line"}
                and target_control_mode in {"station_hold", "station_hold_fast"}
            )
        )
        if self.target is not None and not compatible_target:
            missing.append("incompatible_trajectory_control_mode")
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
        if not self.pid_selected:
            return
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

        # External-IMU roll/pitch and angular rate keep the fast attitude path.
        # Absolute map yaw comes from BodyState so magnetic-heading jumps cannot
        # enter heading feedback or target-frame angular-rate conversion.
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
        control_mode = str(target.control_mode).lower()
        trajectory_phase = str(target.trajectory_phase).lower()
        idle_mode = control_mode == "idle"
        altitude_mode = control_mode == "altitude_hold"
        station_mode = control_mode in {"station_hold", "station_hold_fast"}
        fast_station_mode = control_mode == "station_hold_fast"
        direct_altitude_mode = altitude_mode or station_mode
        if (
            direct_altitude_mode != self.altitude_hold_active
            or station_mode != self.station_hold_active
            or fast_station_mode != self.fast_station_hold_active
            or (
                station_mode
                and trajectory_phase != self.station_trajectory_phase
            )
        ):
            self.reset_controllers()
        self.altitude_hold_active = direct_altitude_mode
        self.station_hold_active = station_mode
        self.fast_station_hold_active = fast_station_mode
        self.station_trajectory_phase = trajectory_phase if station_mode else ""

        if idle_mode:
            # Select+Arm precedes the managed task's explicit Start. Keep a
            # fresh enabled PID command for the authority gate, but never run a
            # hold controller or move a thruster during that hand-off window.
            self.reset_controllers()
            commands = np.zeros(8, dtype=np.float64)
            allocation_residual = 0.0
            allocation_saturation = 0.0
            status_message = "ready; neutral until Start"
        elif direct_altitude_mode:
            # All direct hold modes use the same map-Z actuator-domain PID.
            # Station modes add direct PWM-domain planar-position and heading
            # loops on T5--T8 for any compatible target trajectory. Only fast
            # station hold closes roll/pitch loops on vertical T1--T4.
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
            surge_effort = 0.0
            sway_effort = 0.0
            yaw_effort = 0.0
            yaw_saturated = False
            yaw_integral_state = "inactive"
            horizontal_axis_limit = 0.0
            yaw_axis_limit = 0.0
            horizontal_mix_scale = 1.0
            sway_pid = self.station_sway_pid
            if station_mode:
                trajectory_start_approach = trajectory_phase == "start_approach"
                surge_rate_limit = (
                    self.fast_station_surge_rate_limit
                    if fast_station_mode
                    else self.station_surge_rate_limit
                )
                sway_rate_limit = (
                    self.fast_station_sway_rate_limit
                    if fast_station_mode
                    else self.station_sway_rate_limit
                )
                sway_pid = (
                    self.fast_station_sway_pid
                    if fast_station_mode
                    else self.station_sway_pid
                )
                horizontal_axis_limit = (
                    self.command_limit
                    if fast_station_mode or trajectory_start_approach
                    else min(
                        self.command_limit,
                        self.station_horizontal_axis_limit,
                    )
                )
                yaw_axis_limit = min(
                    self.command_limit,
                    self.station_yaw_axis_limit,
                )
                # The finite 15 s center approach owns its velocity and
                # acceleration profile. Do not clip that planned motion a
                # second time inside the controller. The common live PWM bound
                # below remains the final actuator safety limit.
                desired_planar_velocity = station_velocity_setpoints(
                    target_linear_body[:2],
                    position_error_body[:2],
                    self.station_position_kp,
                    surge_rate_limit,
                    sway_rate_limit,
                    center_approach=trajectory_start_approach,
                )
                desired_surge_velocity = float(desired_planar_velocity[0])
                desired_sway_velocity = float(desired_planar_velocity[1])
                desired_yaw_rate = float(
                    np.clip(
                        target_angular_body[2]
                        + self.station_yaw_heading_kp * orientation_error_body[2],
                        -self.station_yaw_rate_limit,
                        self.station_yaw_rate_limit,
                    )
                )
                target_yaw_rate = float(target_angular_body[2])
                yaw_maneuver_active = (
                    abs(target_yaw_rate)
                    > STATION_YAW_MANEUVER_RATE_THRESHOLD_RPS
                )
                straight_lateral_maneuver = (
                    fast_station_mode
                    and not yaw_maneuver_active
                    and (
                        abs(float(target_linear_body[1]))
                        > STATION_YAW_MANEUVER_RATE_THRESHOLD_RPS
                        or abs(self.fast_station_sway_limited_effort)
                        > FAST_STATION_SURGE_DIRECTION_DEADBAND
                        or abs(float(actual_linear_body[1]))
                        > FAST_STATION_LATERAL_ACTIVE_VELOCITY_MPS
                    )
                )
                if straight_lateral_maneuver:
                    # The measured large-sway run showed the yaw loop repeatedly
                    # reversing at its full +/-0.20 limit. Retain enough heading
                    # authority for straight translation without exciting that
                    # coupled oscillation. Explicit operator yaw keeps the normal
                    # station_yaw_axis_limit above.
                    yaw_axis_limit = min(
                        yaw_axis_limit,
                        self.fast_station_lateral_yaw_axis_limit,
                    )
                yaw_integral_state = "hold"
                if yaw_maneuver_active:
                    self.station_yaw_pid.integral = 0.0
                    self.station_yaw_heading_error_sign = 0
                    yaw_integral_state = "maneuver-disabled"
                elif straight_lateral_maneuver:
                    # The fast lateral translation owns a measured static yaw
                    # feedforward below. Do not let a retained yaw integral
                    # fight it and sustain the observed 2.2 Hz reversals.
                    self.station_yaw_pid.integral = 0.0
                    self.station_yaw_heading_error_sign = 0
                    yaw_integral_state = "lateral-disabled"
                elif self.station_yaw_maneuver_active:
                    self.station_yaw_pid.reset()
                    self.station_yaw_heading_error_sign = 0
                    yaw_integral_state = "hold-reset"
                else:
                    reset_yaw_integral, error_sign = (
                        integral_reset_after_error_crossing(
                            self.station_yaw_heading_error_sign,
                            float(orientation_error_body[2]),
                            self.station_yaw_pwm_ki
                            * self.station_yaw_pid.integral,
                            STATION_YAW_ERROR_CROSSING_HYSTERESIS_RAD,
                        )
                    )
                    self.station_yaw_heading_error_sign = error_sign
                    if reset_yaw_integral:
                        self.station_yaw_pid.integral = 0.0
                        yaw_integral_state = "zero-cross-reset"
                self.station_yaw_maneuver_active = yaw_maneuver_active
                surge_integral_before = self.station_surge_pid.integral
                sway_integral_before = sway_pid.integral
                yaw_integral_before = self.station_yaw_pid.integral
                surge_effort = self.station_surge_pid.step(
                    setpoint=desired_surge_velocity,
                    measurement=float(actual_linear_body[0]),
                    dt=dt,
                    output_limit=horizontal_axis_limit,
                )
                sway_feedforward = (
                    directional_velocity_feedforward(
                        desired_sway_velocity,
                        self.fast_station_sway_pwm_feedforward_positive_gain,
                        self.fast_station_sway_pwm_feedforward_negative_gain,
                        horizontal_axis_limit,
                    )
                    if fast_station_mode
                    else 0.0
                )
                requested_sway_effort = sway_pid.step(
                    setpoint=desired_sway_velocity,
                    measurement=float(actual_linear_body[1]),
                    dt=dt,
                    feedforward=sway_feedforward,
                    output_limit=horizontal_axis_limit,
                )
                if fast_station_mode:
                    self.fast_station_sway_limited_effort = slew_rate_limit(
                        self.fast_station_sway_limited_effort,
                        requested_sway_effort,
                        dt,
                        self.fast_station_sway_effort_slew_rate_per_s,
                    )
                    sway_effort = self.fast_station_sway_limited_effort
                else:
                    self.fast_station_sway_limited_effort = 0.0
                    sway_effort = requested_sway_effort
                yaw_feedforward = (
                    -self.fast_station_sway_yaw_compensation_gain * sway_effort
                    if straight_lateral_maneuver
                    else 0.0
                )
                requested_yaw_effort = self.station_yaw_pid.step(
                    setpoint=desired_yaw_rate,
                    measurement=float(self.imu_angular_velocity_filtered[2]),
                    dt=dt,
                    feedforward=yaw_feedforward,
                    output_limit=yaw_axis_limit,
                    integral_enabled=not (
                        yaw_maneuver_active or straight_lateral_maneuver
                    ),
                )
                if straight_lateral_maneuver:
                    self.fast_station_lateral_yaw_limited_effort = slew_rate_limit(
                        self.fast_station_lateral_yaw_limited_effort,
                        requested_yaw_effort,
                        dt,
                        self.fast_station_lateral_yaw_effort_slew_rate_per_s,
                    )
                    yaw_effort = self.fast_station_lateral_yaw_limited_effort
                else:
                    self.fast_station_lateral_yaw_limited_effort = 0.0
                    yaw_effort = requested_yaw_effort
                yaw_saturated = self.station_yaw_pid.saturated
                _, horizontal_mix_scale = station_horizontal_pwm_mix(
                    surge_effort,
                    sway_effort,
                    yaw_effort,
                    self.command_limit,
                )
                if horizontal_mix_scale < 1.0 - 1e-12:
                    # The individual axis PIDs cannot see saturation caused by
                    # combining axes. Roll back this cycle's integral update so
                    # the common mixer cannot hide horizontal wind-up.
                    self.station_surge_pid.integral = surge_integral_before
                    sway_pid.integral = sway_integral_before
                    self.station_yaw_pid.integral = yaw_integral_before

            applied_surge_effort = surge_effort * horizontal_mix_scale
            applied_sway_effort = sway_effort * horizontal_mix_scale
            applied_yaw_effort = yaw_effort * horizontal_mix_scale

            level_efforts = np.zeros(2, dtype=np.float64)
            level_saturated = False
            pitch_decoupling_effort = 0.0
            altitude_output_limit = self.command_limit
            if fast_station_mode:
                surge_direction = (
                    1
                    if applied_surge_effort
                    > FAST_STATION_SURGE_DIRECTION_DEADBAND
                    else -1
                    if applied_surge_effort
                    < -FAST_STATION_SURGE_DIRECTION_DEADBAND
                    else 0
                )
                if surge_direction != self.fast_station_surge_direction:
                    self.fast_station_pitch_rate_integral_effort = 0.0
                self.fast_station_surge_direction = surge_direction
                if self.fast_station_surge_pitch_decoupling_enabled:
                    pitch_decoupling_effort = surge_pitch_decoupling_effort(
                        applied_surge_effort,
                        self.fast_station_surge_pitch_forward_gain,
                        self.fast_station_surge_pitch_reverse_gain,
                        self.fast_station_surge_pitch_limit,
                    )
                level_limit = min(
                    self.command_limit, self.fast_station_level_pwm_limit
                )
                _, level_base_raw, desired_level_rate, _ = (
                    level_attitude_rate_efforts(
                        orientation_error_body[:2],
                        target_angular_body[:2],
                        self.imu_angular_velocity_filtered[:2],
                        self.fast_station_level_angle_to_rate_kp,
                        self.fast_station_level_rate_kp,
                        self.fast_station_level_rate_limit_rps,
                        level_limit,
                        effort_feedforward=[0.0, pitch_decoupling_effort],
                    )
                )
                self.fast_station_pitch_rate_integral_effort = (
                    conditional_axis_integral_effort(
                        self.fast_station_pitch_rate_integral_effort,
                        float(
                            desired_level_rate[1]
                            - self.imu_angular_velocity_filtered[1]
                        ),
                        self.fast_station_pitch_rate_ki,
                        dt,
                        self.fast_station_pitch_rate_integral_effort_limit,
                        level_base_raw,
                        1,
                        level_limit,
                    )
                )
                level_efforts, _, _, level_saturated = (
                    level_attitude_rate_efforts(
                        orientation_error_body[:2],
                        target_angular_body[:2],
                        self.imu_angular_velocity_filtered[:2],
                        self.fast_station_level_angle_to_rate_kp,
                        self.fast_station_level_rate_kp,
                        self.fast_station_level_rate_limit_rps,
                        level_limit,
                        effort_feedforward=[0.0, pitch_decoupling_effort],
                        effort_integral=[
                            0.0,
                            self.fast_station_pitch_rate_integral_effort,
                        ],
                    )
                )
                # Roll/pitch differential is attitude-priority. The altitude
                # integrator sees only the headroom left by rate feedback,
                # pitch integral, and surge-pitch decoupling, so it cannot
                # wind up behind a saturated individual T1--T4 channel.
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
            # common actuator-domain effort for T1--T4, bounded only by the
            # live unified PWM Limit.
            if station_mode:
                commands = altitude_station_pwm_commands(
                    controller_effort,
                    surge_effort,
                    sway_effort,
                    yaw_effort,
                    self.command_limit,
                    self.altitude_pwm_command_sign,
                    roll_effort=float(level_efforts[0]),
                    pitch_effort=float(level_efforts[1]),
                )
            else:
                commands = altitude_collective_pwm_commands(
                    controller_effort,
                    self.command_limit,
                    self.altitude_pwm_command_sign,
                )
            allocation_residual = 0.0
            allocation_saturation = float(self.altitude_pid.saturated)
            if station_mode:
                horizontal_saturated = (
                    self.station_surge_pid.saturated
                    or sway_pid.saturated
                    or yaw_saturated
                    or horizontal_mix_scale < 1.0 - 1e-12
                    or (
                        self.command_limit > 0.0
                        and np.any(
                            np.abs(commands[4:])
                            >= self.command_limit - 1e-9
                        )
                    )
                )
                allocation_saturation = float(
                    self.altitude_pid.saturated
                    or horizontal_saturated
                    or level_saturated
                )
                if fast_station_mode:
                    status_message = (
                        f"fast station height {target.target_pose.position.z:.3f} m; "
                        "roll/pitch rate cascade "
                        f"{level_efforts[0]:+.3f}/{level_efforts[1]:+.3f}; "
                        "pitch I "
                        f"{self.fast_station_pitch_rate_integral_effort:+.3f}; "
                        f"pitch decoupling {pitch_decoupling_effort:+.3f}; "
                        "surge/sway PID; yaw PI "
                        f"{applied_surge_effort:+.3f}/"
                        f"{applied_sway_effort:+.3f}/"
                        f"{applied_yaw_effort:+.3f}; "
                        "horizontal I "
                        f"{self.station_surge_pwm_ki * self.station_surge_pid.integral:+.3f}/"
                        f"{self.station_sway_pwm_ki * sway_pid.integral:+.3f}; "
                        "yaw I "
                        f"{self.station_yaw_pwm_ki * self.station_yaw_pid.integral:+.3f} "
                        f"({yaw_integral_state}); "
                        f"individual PWM <= {self.pwm_limit_us:.0f} us"
                    )
                else:
                    status_message = (
                        f"station height {target.target_pose.position.z:.3f} m; "
                        "surge/sway PID; yaw PI "
                        f"{applied_surge_effort:+.3f}/"
                        f"{applied_sway_effort:+.3f}/"
                        f"{applied_yaw_effort:+.3f}; "
                        "horizontal I "
                        f"{self.station_surge_pwm_ki * self.station_surge_pid.integral:+.3f}/"
                        f"{self.station_sway_pwm_ki * sway_pid.integral:+.3f}; "
                        "yaw I "
                        f"{self.station_yaw_pwm_ki * self.station_yaw_pid.integral:+.3f} "
                        f"({yaw_integral_state}); "
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
                ["unsupported_control_mode"],
                body_age,
                imu_age,
                target_age,
            )
            return

        command = ThrusterCommand()
        command.header.stamp = now.to_msg()
        command.header.frame_id = "base_link"
        command.action = [
            float(
                np.clip(
                    value * PID_EFFORT_PWM_SPAN_US / ACTION_PWM_SPAN_US,
                    -1.0,
                    1.0,
                )
            )
            for value in commands
        ]
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
        command.action = [0.0] * 8
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
