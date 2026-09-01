"""Managed hold and deterministic planar-trajectory target publisher.

Target generation is deliberately separate from controller selection. The
published ``control_mode`` names the already-selected execution profile; this
node never chooses controller gains or actuator behavior from a trajectory.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from std_srvs.srv import Trigger

from robotcore_interfaces.msg import BodyState, ThrusterCommand, TrajectoryTarget


AUTOMATIC_TRAJECTORY_TYPES = frozenset(
    {
        "spatial_lissajous",
        "circle",
        "racetrack",
        "straight_line",
    }
)


@dataclass
class TrajectorySample:
    """One target sample in world/map coordinates."""

    position: tuple[float, float, float]
    velocity: tuple[float, float, float]
    acceleration: tuple[float, float, float]


class TrajectoryCommandNode(Node):
    """Publish deterministic hold or automatic planar-trajectory targets."""

    def __init__(self):
        super().__init__("trajectory_command_node")
        sensor_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.BEST_EFFORT
        )
        self.declare_parameter("trajectory_type", "hold")
        self.declare_parameter("control_mode", "idle")
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("center_x", 0.0)
        self.declare_parameter("center_y", 0.0)
        self.declare_parameter("center_z", 0.9)
        self.declare_parameter("amp_x", 1.5)
        self.declare_parameter("amp_y", 0.75)
        self.declare_parameter("amp_z", 0.4)
        self.declare_parameter("radius_m", 1.0)
        self.declare_parameter("period_s", 16.0)
        self.declare_parameter("trajectory_speed_mps", 0.1)
        self.declare_parameter("trajectory_ramp_s", 0.0)
        self.declare_parameter("relative_to_initial_pose", False)
        self.declare_parameter("hold_before_motion_s", 0.0)
        self.declare_parameter("attitude_mode", "fixed_identity")
        self.declare_parameter("move_duration_s", 18.0)
        self.declare_parameter("require_pool_bounds", False)
        self.declare_parameter("pool_min_xyz", [0.0, 0.0, 0.0])
        self.declare_parameter("pool_max_xyz", [0.0, 0.0, 0.0])
        self.declare_parameter("trajectory_limits_configured", False)
        self.declare_parameter("max_linear_speed_mps", 0.0)
        self.declare_parameter("manual_vertical_speed_mps", 0.20)
        self.declare_parameter("manual_input_age_s", 0.15)
        self.declare_parameter("imu_topic", "/sensors/external_imu")
        self.declare_parameter("station_linear_input_gain_mps", 0.30)
        self.declare_parameter("station_lateral_input_gain_mps", 0.20)
        self.declare_parameter("station_yaw_input_gain_rps", 0.60)
        self.declare_parameter("max_angular_speed_rps", 0.0)
        self.declare_parameter("attitude_min_rpy_deg", [0.0, 0.0, 0.0])
        self.declare_parameter("attitude_max_rpy_deg", [0.0, 0.0, 0.0])

        self.started_ns = self.get_clock().now().nanoseconds
        self.tracking_started = False
        self.initial_position = None
        self.initial_quaternion = None
        self.latest_position = None
        self.latest_map_yaw = None
        self.latest_quaternion = None
        self.latest_angular_velocity = None
        self.motion_start_position = None
        self.motion_start_quaternion = None
        self.altitude_hold_z = None
        self.altitude_heave_active = False
        self.station_target_position = None
        self.station_target_quaternion = None
        self.station_planar_active = False
        self.station_heave_active = False
        self.station_yaw_active = False
        self.operator_target_input = None
        self.operator_target_input_ns = None
        self.manual_command = None
        self.manual_command_ns = None
        self.envelope_checked = False
        self.envelope_valid = False
        self.envelope_rejection_reason = ""
        self.pub = self.create_publisher(TrajectoryTarget, "/runtime/trajectory_target", 1)
        self.create_subscription(BodyState, "/robot/body_state", self.on_body_state, 1)
        self.create_subscription(
            Imu,
            str(self.get_parameter("imu_topic").value),
            self.on_imu,
            sensor_qos,
        )
        self.create_subscription(
            TwistStamped,
            "/runtime/operator_target_input",
            self.on_operator_target_input,
            1,
        )
        self.create_subscription(
            ThrusterCommand,
            "/control/manual/thruster_cmd",
            self.on_manual_command,
            1,
        )
        self.create_service(
            Trigger, "/runtime/trajectory/reset", self.on_reset_scenario
        )
        self.create_service(
            Trigger, "/runtime/trajectory/stop", self.on_stop_scenario
        )
        self.create_service(
            Trigger, "/runtime/trajectory/validate", self.on_validate_scenario
        )

        rate = float(self.get_parameter("publish_rate_hz").value)
        self.timer = self.create_timer(1.0 / max(rate, 0.1), self.tick)

    def tick(self):
        """Publish the next target using ROS time since node startup."""

        now = self.get_clock().now()
        time_s = self.trajectory_time_s(now.nanoseconds)
        trajectory_type = str(self.get_parameter("trajectory_type").value).lower()
        configured_control_mode = str(
            self.get_parameter("control_mode").value
        ).lower()
        hold_s = max(0.0, float(self.get_parameter("hold_before_motion_s").value))
        effective_time_s = max(0.0, time_s - hold_s)
        if not self.tracking_started:
            sample = self.idle_hold_sample()
            orientation = tuple(
                self.latest_quaternion
                if self.latest_quaternion is not None
                else self.attitude_quaternion("hold", 0.0)
            )
            # This remains a valid stationary target for the authority
            # freshness gate, but it is not an instruction to actuate. The
            # PID command stays neutral until the explicit Start/reset.
            published_trajectory_type = "idle"
            published_control_mode = "idle"
            published_phase = "idle"
            target_valid = self.idle_target_is_valid(sample.position)
            angular_velocity = (0.0, 0.0, 0.0)
            angular_acceleration = (0.0, 0.0, 0.0)
        elif trajectory_type == "hold" and configured_control_mode == "altitude_hold":
            sample = self.altitude_hold_sample(now.nanoseconds)
            orientation = tuple(
                self.latest_quaternion
                if self.latest_quaternion is not None
                else self.attitude_quaternion("hold", 0.0)
            )
            published_trajectory_type = trajectory_type
            published_control_mode = configured_control_mode
            published_phase = "hold"
            target_valid = self.target_is_valid(sample.position)
            angular_velocity = (0.0, 0.0, 0.0)
            angular_acceleration = (0.0, 0.0, 0.0)
        elif trajectory_type == "hold" and configured_control_mode in {
            "station_hold",
            "station_hold_fast",
            "rl_policy",
        }:
            sample, orientation, angular_velocity = self.station_hold_target(
                now.nanoseconds
            )
            published_trajectory_type = trajectory_type
            published_control_mode = configured_control_mode
            published_phase = "hold"
            target_valid = self.target_is_valid(sample.position)
            angular_acceleration = (0.0, 0.0, 0.0)
        elif trajectory_type in AUTOMATIC_TRAJECTORY_TYPES and time_s < hold_s:
            move_s = max(
                0.1, float(self.get_parameter("move_duration_s").value)
            )
            move_time_s = min(time_s, move_s)
            turn_time_s = max(0.0, time_s - move_s)
            sample = self.sample("move_to_trajectory_start", move_time_s)
            orientation, angular_velocity, angular_acceleration = (
                self.sample_attitude(
                    "move_to_trajectory_start", turn_time_s
                )
            )
            published_trajectory_type = trajectory_type
            published_control_mode = configured_control_mode
            published_phase = (
                "start_approach" if time_s < move_s else "heading_alignment"
            )
            target_valid = self.target_is_valid(sample.position)
        elif trajectory_type in AUTOMATIC_TRAJECTORY_TYPES:
            sample = self.sample(trajectory_type, effective_time_s)
            orientation, angular_velocity, angular_acceleration = self.sample_attitude(
                trajectory_type, effective_time_s
            )
            published_trajectory_type = trajectory_type
            published_control_mode = configured_control_mode
            published_phase = "tracking"
            target_valid = self.target_is_valid(sample.position)
        else:
            sample = self.idle_hold_sample()
            orientation = tuple(
                self.latest_quaternion
                if self.latest_quaternion is not None
                else self.attitude_quaternion("hold", 0.0)
            )
            angular_velocity = (0.0, 0.0, 0.0)
            angular_acceleration = (0.0, 0.0, 0.0)
            published_trajectory_type = trajectory_type
            published_control_mode = configured_control_mode
            published_phase = "invalid"
            target_valid = False

        if self.tracking_started and not self.control_mode_is_compatible(
            trajectory_type, configured_control_mode
        ):
            target_valid = False

        # Every published pose target is level by definition.  Measured tilt
        # is feedback, never a target: keep only the requested map yaw and
        # reject roll/pitch angular feed-forward at the publication boundary.
        # This applies equally to idle, holds, manual position commands,
        # automatic trajectories, and the move-to-start prelude.
        orientation = tuple(self.level_heading_quaternion(orientation))
        angular_velocity = (0.0, 0.0, float(angular_velocity[2]))
        angular_acceleration = (0.0, 0.0, float(angular_acceleration[2]))

        msg = TrajectoryTarget()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = "map"
        msg.target_pose.position.x = sample.position[0]
        msg.target_pose.position.y = sample.position[1]
        msg.target_pose.position.z = sample.position[2]
        (
            msg.target_pose.orientation.w,
            msg.target_pose.orientation.x,
            msg.target_pose.orientation.y,
            msg.target_pose.orientation.z,
        ) = orientation
        msg.target_twist.linear.x = sample.velocity[0]
        msg.target_twist.linear.y = sample.velocity[1]
        msg.target_twist.linear.z = sample.velocity[2]
        msg.target_accel.linear.x = sample.acceleration[0]
        msg.target_accel.linear.y = sample.acceleration[1]
        msg.target_accel.linear.z = sample.acceleration[2]
        msg.target_twist.angular.x = angular_velocity[0]
        msg.target_twist.angular.y = angular_velocity[1]
        msg.target_twist.angular.z = angular_velocity[2]
        msg.target_accel.angular.x = angular_acceleration[0]
        msg.target_accel.angular.y = angular_acceleration[1]
        msg.target_accel.angular.z = angular_acceleration[2]
        msg.trajectory_type = published_trajectory_type
        msg.control_mode = published_control_mode
        msg.trajectory_phase = published_phase
        msg.time_s = float(time_s)
        msg.valid = target_valid
        self.pub.publish(msg)

    def trajectory_time_s(self, now_ns):
        """Keep Target time frozen until the explicit Start/reset service."""

        if not self.tracking_started:
            return 0.0
        return max(0.0, (int(now_ns) - self.started_ns) * 1e-9)

    def on_body_state(self, msg: BodyState):
        """Track EKF position and its disturbance-free absolute map heading."""

        if not (msg.state_valid or msg.position_estimated):
            return
        position = np.asarray(
            [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z],
            dtype=np.float64,
        )
        if not np.all(np.isfinite(position)):
            return
        self.latest_position = position
        body_quaternion = np.asarray(
            [
                msg.pose.orientation.w,
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
            ],
            dtype=np.float64,
        )
        body_norm = float(np.linalg.norm(body_quaternion))
        if np.isfinite(body_norm) and body_norm > 1e-9:
            self.latest_map_yaw = float(
                self.quaternion_to_rpy(body_quaternion / body_norm)[2]
            )
        if self.initial_position is None:
            self.initial_position = position.copy()
            self.envelope_checked = False
            self.get_logger().info("Captured trusted initial position for relative trajectory")

    def on_imu(self, msg: Imu):
        """Combine external-IMU tilt/rates with BodyState absolute yaw."""

        quaternion = np.asarray(
            [
                msg.orientation.w,
                msg.orientation.x,
                msg.orientation.y,
                msg.orientation.z,
            ],
            dtype=np.float64,
        )
        norm = float(np.linalg.norm(quaternion))
        if (
            msg.header.frame_id != "base_link"
            or msg.orientation_covariance[0] < 0.0
            or not np.isfinite(norm)
            or norm <= 1e-9
            or self.latest_map_yaw is None
        ):
            return
        imu_rpy = self.quaternion_to_rpy(quaternion / norm)
        self.latest_quaternion = self.rpy_quaternion(
            float(imu_rpy[0]), float(imu_rpy[1]), self.latest_map_yaw
        )
        angular_velocity = np.asarray(
            [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z],
            dtype=np.float64,
        )
        self.latest_angular_velocity = (
            angular_velocity if np.all(np.isfinite(angular_velocity)) else None
        )
        if self.initial_quaternion is None:
            self.initial_quaternion = self.latest_quaternion.copy()

    def on_manual_command(self, msg: ThrusterCommand):
        if msg.source != "web_operator":
            return
        self.manual_command = msg
        self.manual_command_ns = self.get_clock().now().nanoseconds

    def on_operator_target_input(self, msg: TwistStamped):
        self.operator_target_input = msg
        self.operator_target_input_ns = self.get_clock().now().nanoseconds

    def current_manual_heave(self, now_ns):
        maximum_age = float(self.get_parameter("manual_input_age_s").value)
        age = (
            math.inf
            if self.manual_command_ns is None
            else (int(now_ns) - self.manual_command_ns) * 1e-9
        )
        if (
            self.manual_command is None
            or age > maximum_age
            or not self.manual_command.enable
            or self.manual_command.source != "web_operator"
        ):
            return 0.0
        values = np.asarray(self.manual_command.action, dtype=np.float64)
        if values.shape != (8,) or not np.all(np.isfinite(values)):
            return 0.0
        return float(np.clip(np.mean(values[:4]), -1.0, 1.0))

    def altitude_hold_sample(self, now_ns):
        """Command vertical speed while held; latch measured height on release."""

        if self.latest_position is None:
            position = self.motion_start_position
        else:
            position = self.latest_position
        if position is None:
            position = np.zeros(3, dtype=np.float64)

        heave = self.current_manual_heave(now_ns)
        heave_active = not math.isclose(heave, 0.0, abs_tol=1e-3)

        # Do not accumulate a setpoint ahead of the vehicle. While the stick
        # is held, measured Z remains the pose target and the stick commands
        # vertical velocity. On the release edge, measured Z is retained as
        # the new fixed height and requested vertical velocity becomes zero.
        if heave_active or self.altitude_heave_active:
            self.altitude_hold_z = float(position[2])
        target_z = (
            float(position[2])
            if self.altitude_hold_z is None
            else self.altitude_hold_z
        )
        self.altitude_heave_active = heave_active
        # Standard-gamepad stick-up is a negative browser axis. Convert it to
        # positive map FLU vertical velocity; the PID node owns the verified
        # sign conversion from FLU effort to the installed thrusters.
        velocity_z = -heave * abs(
            float(self.get_parameter("manual_vertical_speed_mps").value)
        )

        return TrajectorySample(
            position=(float(position[0]), float(position[1]), target_z),
            velocity=(0.0, 0.0, float(velocity_z)),
            acceleration=(0.0, 0.0, 0.0),
        )

    def current_station_input(self, now_ns):
        maximum_age = float(self.get_parameter("manual_input_age_s").value)
        age = (
            math.inf
            if self.operator_target_input_ns is None
            else (int(now_ns) - self.operator_target_input_ns) * 1e-9
        )
        if self.operator_target_input is None or age > maximum_age:
            return np.zeros(4, dtype=np.float64)
        twist = self.operator_target_input.twist
        return np.clip(
            np.asarray(
                [twist.linear.x, twist.linear.y, twist.linear.z, twist.angular.z],
                dtype=np.float64,
            ),
            -1.0,
            1.0,
        )

    def station_hold_target(self, now_ns):
        """Publish Station Hold Fast velocity and latch pose on stick release."""

        if self.station_target_position is None:
            source = self.motion_start_position
            if source is None:
                source = self.latest_position
            if source is None:
                source = np.zeros(3, dtype=np.float64)
            self.station_target_position = np.asarray(source, dtype=np.float64).copy()
        if self.station_target_quaternion is None:
            source = self.motion_start_quaternion
            if source is None:
                source = self.latest_quaternion
            if source is None:
                source = np.asarray([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
            self.station_target_quaternion = self.level_heading_quaternion(source)

        command = self.current_station_input(now_ns)
        # Every gamepad pose target uses the Station Hold Fast convention.
        # Controller selection does not alter stick axes or target semantics:
        # left-stick Y/X are forward/yaw, right-stick X/Y are lateral/height.
        forward, left, up, yaw_ccw = (float(value) for value in command)
        planar_active = not np.allclose(command[:2], 0.0, atol=1e-3)
        heave_active = not math.isclose(up, 0.0, abs_tol=1e-3)
        yaw_active = not math.isclose(yaw_ccw, 0.0, abs_tol=1e-3)

        # While a rate command is active, keep the position target on the
        # measured vehicle and drive motion with velocity feed-forward.  On
        # the first neutral sample, latch the final measured pose.  This is
        # the Station Hold Fast contract: the green target cannot integrate
        # away from a slower vehicle and become an unreachable moving goal.
        if self.latest_position is not None:
            if planar_active or self.station_planar_active:
                self.station_target_position[:2] = self.latest_position[:2]
            if heave_active or self.station_heave_active:
                self.station_target_position[2] = self.latest_position[2]
        if self.latest_quaternion is not None and (
            yaw_active or self.station_yaw_active
        ):
            # The left-stick horizontal axis owns yaw. Roll and pitch
            # remain level instead of latching a transient measured tilt.
            measured_yaw = self.quaternion_to_rpy(self.latest_quaternion)[2]
            self.station_target_quaternion = self.rpy_quaternion(
                0.0, 0.0, measured_yaw
            )

        self.station_planar_active = planar_active
        self.station_heave_active = heave_active
        self.station_yaw_active = yaw_active

        linear_gain = abs(
            float(self.get_parameter("station_linear_input_gain_mps").value)
        )
        lateral_gain = abs(
            float(self.get_parameter("station_lateral_input_gain_mps").value)
        )
        vertical_gain = abs(
            float(self.get_parameter("manual_vertical_speed_mps").value)
        )
        yaw_gain = abs(
            float(self.get_parameter("station_yaw_input_gain_rps").value)
        )
        # Browser/gamepad convention: right stick down is positive.  Map FLU
        # uses +Z up, so the target-height rate must carry the opposite sign.
        target_vertical_velocity = -up * vertical_gain
        current_quaternion = (
            self.latest_quaternion
            if self.latest_quaternion is not None
            else self.station_target_quaternion
        )
        # Operator X/Y commands are horizontal map-plane commands.  Use the
        # current heading only, so measured roll/pitch cannot tilt or scale a
        # requested horizontal direction.
        rotation = self.rotation_matrix(
            self.level_heading_quaternion(current_quaternion)
        )
        planar_world = rotation @ np.asarray(
            [forward * linear_gain, left * lateral_gain, 0.0],
            dtype=np.float64,
        )
        angular_world = rotation @ np.asarray(
            [0.0, 0.0, yaw_ccw * yaw_gain], dtype=np.float64
        )
        if bool(self.get_parameter("require_pool_bounds").value):
            minimum = np.asarray(
                self.get_parameter("pool_min_xyz").value, dtype=np.float64
            )
            maximum = np.asarray(
                self.get_parameter("pool_max_xyz").value, dtype=np.float64
            )
            if minimum.shape == (3,) and maximum.shape == (3,):
                self.station_target_position[:] = np.clip(
                    self.station_target_position,
                    minimum,
                    maximum,
                )
        sample = TrajectorySample(
            position=tuple(float(value) for value in self.station_target_position),
            velocity=(
                float(planar_world[0]),
                float(planar_world[1]),
                float(target_vertical_velocity),
            ),
            acceleration=(0.0, 0.0, 0.0),
        )
        return (
            sample,
            tuple(float(value) for value in self.station_target_quaternion),
            tuple(float(value) for value in angular_world),
        )

    def on_reset_scenario(self, _request, response):
        trajectory_type = str(self.get_parameter("trajectory_type").value).lower()
        control_mode = str(self.get_parameter("control_mode").value).lower()
        if not self.control_mode_is_compatible(trajectory_type, control_mode):
            response.success = False
            response.message = (
                f"trajectory {trajectory_type} is incompatible with control mode "
                f"{control_mode}"
            )
            return response
        if (
            trajectory_type in {"hold", *AUTOMATIC_TRAJECTORY_TYPES}
            and self.motion_start_position is None
        ):
            response.success = False
            response.message = f"{trajectory_type} requires a validated current body pose"
            return response
        if (
            trajectory_type in {"hold", *AUTOMATIC_TRAJECTORY_TYPES}
            and self.motion_start_quaternion is None
        ):
            response.success = False
            response.message = f"{trajectory_type} requires a validated current attitude"
            return response
        if trajectory_type == "hold" and control_mode == "altitude_hold":
            self.altitude_hold_z = float(self.get_parameter("center_z").value)
            self.altitude_heave_active = False
        elif trajectory_type == "hold" and control_mode in {
            "station_hold",
            "station_hold_fast",
            "rl_policy",
        }:
            self.station_target_position = self.motion_start_position.copy()
            # Standard Station Hold retains its configured absolute height.
            # Station Hold Fast and RL+None instead latch the measured height
            # at Start; their right-stick rate then updates and re-latches it.
            if control_mode == "station_hold":
                self.station_target_position[2] = float(
                    self.get_parameter("center_z").value
                )
            self.station_target_quaternion = self.level_heading_quaternion(
                self.motion_start_quaternion
            )
            self.station_planar_active = False
            self.station_heave_active = False
            self.station_yaw_active = False
        self.started_ns = self.get_clock().now().nanoseconds
        self.tracking_started = True
        response.success = True
        response.message = "trajectory tracking started at 0.00 s"
        return response

    def on_stop_scenario(self, _request, response):
        self.tracking_started = False
        self.started_ns = self.get_clock().now().nanoseconds
        self.motion_start_position = None
        self.motion_start_quaternion = None
        self.altitude_hold_z = None
        self.altitude_heave_active = False
        self.station_target_position = None
        self.station_target_quaternion = None
        self.station_planar_active = False
        self.station_heave_active = False
        self.station_yaw_active = False
        self.operator_target_input = None
        self.operator_target_input_ns = None
        self.manual_command = None
        self.manual_command_ns = None
        response.success = True
        response.message = "trajectory ready; target time held at 0.00 s"
        return response

    def on_validate_scenario(self, _request, response):
        trajectory_type = str(self.get_parameter("trajectory_type").value).lower()
        control_mode = str(self.get_parameter("control_mode").value).lower()
        if not self.control_mode_is_compatible(trajectory_type, control_mode):
            self.envelope_valid = False
            self.envelope_checked = True
            response.success = False
            response.message = (
                f"trajectory {trajectory_type} is incompatible with control mode "
                f"{control_mode}"
            )
            return response
        if trajectory_type in {"hold", *AUTOMATIC_TRAJECTORY_TYPES}:
            if self.latest_position is None or self.latest_quaternion is None:
                self.envelope_valid = False
                self.envelope_checked = True
                response.success = False
                response.message = f"{trajectory_type} requires a current valid body pose"
                return response
            # Latch exactly the start point used by both validation and reset;
            # later estimator updates cannot change an already-approved path.
            self.motion_start_position = self.latest_position.copy()
            self.motion_start_quaternion = self.latest_quaternion.copy()
        else:
            self.motion_start_position = None
            self.motion_start_quaternion = None
        self.envelope_valid = self.validate_scenario_envelope()
        self.envelope_checked = True
        response.success = bool(self.envelope_valid)
        response.message = (
            "trajectory envelope is inside all configured limits"
            if self.envelope_valid
            else self.envelope_rejection_reason
            or "trajectory envelope or safety limits are invalid"
        )
        return response

    @staticmethod
    def control_mode_is_compatible(trajectory_type: str, control_mode: str) -> bool:
        """Keep target generation independent while rejecting unsafe pairings."""

        trajectory_type = str(trajectory_type).lower()
        control_mode = str(control_mode).lower()
        if trajectory_type == "hold":
            return control_mode in {
                "altitude_hold",
                "station_hold",
                "station_hold_fast",
                "rl_policy",
            }
        if trajectory_type in AUTOMATIC_TRAJECTORY_TYPES:
            return control_mode in {
                "station_hold",
                "station_hold_fast",
                "rl_policy",
            }
        return False

    def sample(self, trajectory_type: str, time_s: float) -> TrajectorySample:
        """Evaluate a managed hold, fixed-start approach, or automatic path."""

        relative = bool(self.get_parameter("relative_to_initial_pose").value)
        motion_start_position = getattr(self, "motion_start_position", None)
        relative_origin = (
            motion_start_position
            if motion_start_position is not None
            else self.initial_position
        )
        if relative and relative_origin is not None:
            center = tuple(float(value) for value in relative_origin)
        else:
            center = (
                float(self.get_parameter("center_x").value),
                float(self.get_parameter("center_y").value),
                float(self.get_parameter("center_z").value),
            )
        if trajectory_type in {"move_to_hold", "move_to_trajectory_start"}:
            start = self.motion_start_position
            if start is None:
                start = self.latest_position
            if start is None:
                start = np.asarray(center, dtype=np.float64)
            if trajectory_type == "move_to_trajectory_start":
                active_type = str(
                    self.get_parameter("trajectory_type").value
                ).lower()
                goal = np.asarray(
                    self.sample(active_type, 0.0).position,
                    dtype=np.float64,
                )
            else:
                goal = np.asarray(center, dtype=np.float64)
            displacement = goal - np.asarray(start, dtype=np.float64)
            duration = max(0.1, float(self.get_parameter("move_duration_s").value))
            tau = float(np.clip(time_s / duration, 0.0, 1.0))
            blend = 10.0 * tau**3 - 15.0 * tau**4 + 6.0 * tau**5
            blend_rate = (30.0 * tau**2 - 60.0 * tau**3 + 30.0 * tau**4) / duration
            blend_acceleration = (
                60.0 * tau - 180.0 * tau**2 + 120.0 * tau**3
            ) / duration**2
            position = np.asarray(start, dtype=np.float64) + blend * displacement
            velocity = blend_rate * displacement
            acceleration = blend_acceleration * displacement
            return TrajectorySample(
                position=tuple(float(value) for value in position),
                velocity=tuple(float(value) for value in velocity),
                acceleration=tuple(float(value) for value in acceleration),
            )
        if trajectory_type in AUTOMATIC_TRAJECTORY_TYPES:
            offset, velocity, acceleration, _yaw = self.trajectory_kinematics(
                trajectory_type, time_s
            )
        else:
            # Unsupported external names remain stationary and are marked
            # invalid by control_mode_is_compatible() before publication.
            offset = (0.0, 0.0, 0.0)
            velocity = (0.0, 0.0, 0.0)
            acceleration = (0.0, 0.0, 0.0)

        position = (
            center[0] + offset[0],
            center[1] + offset[1],
            center[2] + offset[2],
        )
        return TrajectorySample(position=position, velocity=velocity, acceleration=acceleration)

    def phase_kinematics(self, time_s: float, distance_per_phase: float):
        """Return a ramped phase capped by the configured linear path speed."""

        path_speed = abs(float(self.get_parameter("trajectory_speed_mps").value))
        distance_per_phase = abs(float(distance_per_phase))
        if path_speed <= 0.0 or distance_per_phase <= 1.0e-9:
            return 0.0, 0.0, 0.0
        phase_time, time_rate, time_acceleration = self.ramped_time_kinematics(
            time_s
        )
        phase = path_speed * phase_time / distance_per_phase
        phase_dot = path_speed * time_rate / distance_per_phase
        phase_ddot = path_speed * time_acceleration / distance_per_phase
        return phase, phase_dot, phase_ddot

    def ramped_time_kinematics(self, time_s: float):
        """Return smoothly started effective time and its first derivatives."""

        ramp_s = max(0.0, float(self.get_parameter("trajectory_ramp_s").value))
        if 0.0 < ramp_s and time_s < ramp_s:
            ramp_fraction = float(np.clip(time_s / ramp_s, 0.0, 1.0))
            phase_time = ramp_s * (
                ramp_fraction**3 - 0.5 * ramp_fraction**4
            )
            phase_rate = 3.0 * ramp_fraction**2 - 2.0 * ramp_fraction**3
            phase_acceleration = (
                6.0 * ramp_fraction - 6.0 * ramp_fraction**2
            ) / ramp_s
        else:
            phase_time = time_s - 0.5 * ramp_s
            phase_rate = 1.0
            phase_acceleration = 0.0
        return phase_time, phase_rate, phase_acceleration

    @staticmethod
    def compose_kinematics(path_offset, path_first, path_second, phase_dot, phase_ddot):
        offset = np.asarray(path_offset, dtype=np.float64)
        first = np.asarray(path_first, dtype=np.float64)
        second = np.asarray(path_second, dtype=np.float64)
        velocity = first * phase_dot
        acceleration = second * phase_dot**2 + first * phase_ddot
        yaw = math.atan2(float(first[1]), float(first[0]))
        return tuple(offset), tuple(velocity), tuple(acceleration), yaw

    def trajectory_kinematics(self, trajectory_type: str, time_s: float):
        """Dispatch one configured automatic path in the horizontal plane."""

        trajectory_type = str(trajectory_type).lower()
        if trajectory_type == "spatial_lissajous":
            return self.spatial_lissajous_kinematics(time_s)
        if trajectory_type == "circle":
            return self.circle_kinematics(time_s)
        if trajectory_type == "racetrack":
            return self.racetrack_kinematics(time_s)
        if trajectory_type == "straight_line":
            return self.straight_line_kinematics(time_s)
        zeros = (0.0, 0.0, 0.0)
        return zeros, zeros, zeros, 0.0

    def spatial_lissajous_kinematics(self, time_s: float):
        """Return a 1:2:3 spatial Lissajous at constant total path speed."""

        amp_x = abs(float(self.get_parameter("amp_x").value))
        amp_y = abs(float(self.get_parameter("amp_y").value))
        amp_z = abs(float(self.get_parameter("amp_z").value))
        path_speed = abs(
            float(self.get_parameter("trajectory_speed_mps").value)
        )
        phases, cumulative_length = self.lissajous_arc_table(
            amp_x, amp_y, amp_z
        )
        lap_length = float(cumulative_length[-1])
        if lap_length <= 1.0e-9 or path_speed <= 0.0:
            zeros = (0.0, 0.0, 0.0)
            return zeros, zeros, zeros, 0.0

        effective_time, time_rate, time_acceleration = (
            self.ramped_time_kinematics(time_s)
        )
        distance = path_speed * effective_time
        lap_distance = distance % lap_length
        phase = float(np.interp(lap_distance, cumulative_length, phases))

        path_offset = np.asarray(
            [
                amp_x * math.sin(phase),
                amp_y * math.sin(2.0 * phase),
                amp_z * math.sin(3.0 * phase),
            ],
            dtype=np.float64,
        )
        path_first = np.asarray(
            [
                amp_x * math.cos(phase),
                2.0 * amp_y * math.cos(2.0 * phase),
                3.0 * amp_z * math.cos(3.0 * phase),
            ],
            dtype=np.float64,
        )
        path_second = np.asarray(
            [
                -amp_x * math.sin(phase),
                -4.0 * amp_y * math.sin(2.0 * phase),
                -9.0 * amp_z * math.sin(3.0 * phase),
            ],
            dtype=np.float64,
        )
        derivative_norm = max(1.0e-9, float(np.linalg.norm(path_first)))
        distance_rate = path_speed * time_rate
        distance_acceleration = path_speed * time_acceleration
        phase_dot = distance_rate / derivative_norm
        phase_ddot = (
            distance_acceleration / derivative_norm
            - float(np.dot(path_first, path_second))
            * distance_rate**2
            / derivative_norm**4
        )
        return self.compose_kinematics(
            path_offset,
            path_first,
            path_second,
            phase_dot,
            phase_ddot,
        )

    def lissajous_arc_table(self, amp_x: float, amp_y: float, amp_z: float):
        """Build and cache phase-to-arc-length data for one 1:2:3 lap."""

        key = (float(amp_x), float(amp_y), float(amp_z))
        cached = getattr(self, "_lissajous_arc_cache", None)
        if cached is not None and cached[0] == key:
            return cached[1], cached[2]

        phases = np.linspace(0.0, 2.0 * math.pi, 4097, dtype=np.float64)
        derivatives = np.column_stack(
            (
                amp_x * np.cos(phases),
                2.0 * amp_y * np.cos(2.0 * phases),
                3.0 * amp_z * np.cos(3.0 * phases),
            )
        )
        derivative_norms = np.linalg.norm(derivatives, axis=1)
        phase_step = phases[1] - phases[0]
        segment_lengths = (
            0.5
            * (derivative_norms[:-1] + derivative_norms[1:])
            * phase_step
        )
        cumulative_length = np.concatenate(
            (np.zeros(1, dtype=np.float64), np.cumsum(segment_lengths))
        )
        self._lissajous_arc_cache = (key, phases, cumulative_length)
        return phases, cumulative_length

    def circle_kinematics(self, time_s: float):
        """Return a counter-clockwise circle starting with map-forward tangent."""

        radius = abs(float(self.get_parameter("radius_m").value))
        phase, phase_dot, phase_ddot = self.phase_kinematics(time_s, radius)
        return self.compose_kinematics(
            (radius * math.sin(phase), -radius * math.cos(phase), 0.0),
            (radius * math.cos(phase), radius * math.sin(phase), 0.0),
            (-radius * math.sin(phase), radius * math.cos(phase), 0.0),
            phase_dot,
            phase_ddot,
        )

    def racetrack_kinematics(self, time_s: float):
        """Return a constant-path-speed stadium/racetrack elongated along map X."""

        radius = max(1e-6, abs(float(self.get_parameter("amp_y").value)))
        outer_half_length = max(
            radius, abs(float(self.get_parameter("amp_x").value))
        )
        straight_half_length = outer_half_length - radius
        lap_length = 4.0 * straight_half_length + 2.0 * math.pi * radius
        distance_per_phase = lap_length / (2.0 * math.pi)
        phase, phase_dot, phase_ddot = self.phase_kinematics(
            time_s, distance_per_phase
        )
        distance = (phase % (2.0 * math.pi)) * lap_length / (2.0 * math.pi)

        straight_length = 2.0 * straight_half_length
        if distance < straight_length:
            offset = (-straight_half_length + distance, -radius, 0.0)
            tangent = (1.0, 0.0, 0.0)
            curvature = (0.0, 0.0, 0.0)
        elif distance < straight_length + math.pi * radius:
            angle = -0.5 * math.pi + (distance - straight_length) / radius
            offset = (
                straight_half_length + radius * math.cos(angle),
                radius * math.sin(angle),
                0.0,
            )
            tangent = (-math.sin(angle), math.cos(angle), 0.0)
            curvature = (-math.cos(angle) / radius, -math.sin(angle) / radius, 0.0)
        elif distance < 2.0 * straight_length + math.pi * radius:
            progress = distance - straight_length - math.pi * radius
            offset = (straight_half_length - progress, radius, 0.0)
            tangent = (-1.0, 0.0, 0.0)
            curvature = (0.0, 0.0, 0.0)
        else:
            angle = (
                0.5 * math.pi
                + (distance - 2.0 * straight_length - math.pi * radius) / radius
            )
            offset = (
                -straight_half_length + radius * math.cos(angle),
                radius * math.sin(angle),
                0.0,
            )
            tangent = (-math.sin(angle), math.cos(angle), 0.0)
            curvature = (-math.cos(angle) / radius, -math.sin(angle) / radius, 0.0)

        distance_per_phase = lap_length / (2.0 * math.pi)
        first = np.asarray(tangent, dtype=np.float64) * distance_per_phase
        second = np.asarray(curvature, dtype=np.float64) * distance_per_phase**2
        return self.compose_kinematics(
            offset, first, second, phase_dot, phase_ddot
        )

    def straight_line_kinematics(self, time_s: float):
        """Return a smooth out-and-back line while keeping map-forward heading."""

        half_length = abs(float(self.get_parameter("amp_x").value))
        # A sinusoidal reversal avoids a discontinuous velocity command.  Scaling
        # phase by half_length makes trajectory_speed_mps the peak linear speed.
        phase, phase_dot, phase_ddot = self.phase_kinematics(time_s, half_length)
        offset = (-half_length * math.cos(phase), 0.0, 0.0)
        first = (half_length * math.sin(phase), 0.0, 0.0)
        second = (half_length * math.cos(phase), 0.0, 0.0)
        sample = self.compose_kinematics(
            offset, first, second, phase_dot, phase_ddot
        )
        return sample[0], sample[1], sample[2], 0.0

    def idle_hold_sample(self) -> TrajectorySample:
        """Hold the latest measured position until an approved task starts."""

        if self.latest_position is not None:
            position = tuple(float(value) for value in self.latest_position)
        else:
            position = (
                float(self.get_parameter("center_x").value),
                float(self.get_parameter("center_y").value),
                float(self.get_parameter("center_z").value),
            )
        zeros = (0.0, 0.0, 0.0)
        return TrajectorySample(position=position, velocity=zeros, acceleration=zeros)

    def idle_target_is_valid(self, position):
        """Validate only the measured idle hold point, not a pending scenario."""

        if self.latest_position is None:
            return False
        if not bool(self.get_parameter("require_pool_bounds").value):
            return True
        minimum = np.asarray(self.get_parameter("pool_min_xyz").value, dtype=np.float64)
        maximum = np.asarray(self.get_parameter("pool_max_xyz").value, dtype=np.float64)
        point = np.asarray(position, dtype=np.float64)
        return bool(
            minimum.shape == (3,)
            and maximum.shape == (3,)
            and np.all(minimum < maximum)
            and np.all(point >= minimum)
            and np.all(point <= maximum)
        )

    def sample_attitude(self, trajectory_type: str, time_s: float):
        """Return target quaternion plus world-frame angular velocity/acceleration."""

        quaternion = self.attitude_quaternion(trajectory_type, time_s)
        step = 1.0 / max(20.0, float(self.get_parameter("publish_rate_hz").value))
        omega = self.angular_velocity_at(trajectory_type, time_s, step)
        omega_before = self.angular_velocity_at(
            trajectory_type, max(0.0, time_s - step), step
        )
        omega_after = self.angular_velocity_at(trajectory_type, time_s + step, step)
        acceleration = (omega_after - omega_before) / (2.0 * step)
        return tuple(quaternion), tuple(omega), tuple(acceleration)

    def attitude_quaternion(self, trajectory_type: str, time_s: float):
        if trajectory_type == "move_to_trajectory_start":
            start = next(
                (
                    quaternion
                    for quaternion in (
                        getattr(self, "motion_start_quaternion", None),
                        getattr(self, "latest_quaternion", None),
                        getattr(self, "initial_quaternion", None),
                    )
                    if quaternion is not None
                ),
                np.asarray([1.0, 0.0, 0.0, 0.0], dtype=np.float64),
            )
            duration = max(
                0.1,
                float(self.get_parameter("hold_before_motion_s").value)
                - float(self.get_parameter("move_duration_s").value),
            )
            fraction = float(np.clip(time_s / duration, 0.0, 1.0))
            blend = 10.0 * fraction**3 - 15.0 * fraction**4 + 6.0 * fraction**5
            target_type = str(self.get_parameter("trajectory_type").value).lower()
            _offset, _velocity, _acceleration, start_yaw = self.trajectory_kinematics(
                target_type, 0.0
            )
            goal = self.rpy_quaternion(0.0, 0.0, start_yaw)
            return self.quaternion_slerp(start, goal, blend)
        if trajectory_type in AUTOMATIC_TRAJECTORY_TYPES:
            _offset, _velocity, _acceleration, yaw = self.trajectory_kinematics(
                trajectory_type, time_s
            )
            return self.rpy_quaternion(0.0, 0.0, yaw)

        mode = str(self.get_parameter("attitude_mode").value).lower()
        if mode == "hold_initial":
            # A relocalization can rotate the IMU world-heading reference after
            # process startup. Prefer the attitude latched by the current
            # Validate request; the startup quaternion is only a fallback.
            base = next(
                (
                    quaternion
                    for quaternion in (
                        getattr(self, "motion_start_quaternion", None),
                        getattr(self, "latest_quaternion", None),
                        getattr(self, "initial_quaternion", None),
                    )
                    if quaternion is not None
                ),
                np.asarray([1.0, 0.0, 0.0, 0.0], dtype=np.float64),
            )
        elif (
            bool(self.get_parameter("relative_to_initial_pose").value)
            and self.initial_quaternion is not None
        ):
            base = self.initial_quaternion
        else:
            base = np.asarray([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
        if mode == "fixed_identity":
            return np.asarray([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
        return base

    def angular_velocity_at(self, trajectory_type: str, time_s: float, step: float):
        before = self.attitude_quaternion(
            trajectory_type, max(0.0, time_s - step)
        )
        current = self.attitude_quaternion(trajectory_type, time_s)
        after = self.attitude_quaternion(trajectory_type, time_s + step)
        rotation_before = self.rotation_matrix(before)
        rotation_current = self.rotation_matrix(current)
        rotation_after = self.rotation_matrix(after)
        rotation_dot = (rotation_after - rotation_before) / (2.0 * step)
        skew = rotation_dot @ rotation_current.T
        return np.asarray([skew[2, 1], skew[0, 2], skew[1, 0]], dtype=np.float64)

    def target_is_valid(self, position):
        relative = bool(self.get_parameter("relative_to_initial_pose").value)
        if (
            relative
            and self.motion_start_position is None
            and self.initial_position is None
        ):
            return False
        if not bool(self.get_parameter("require_pool_bounds").value):
            return True
        if not self.envelope_checked:
            self.envelope_valid = self.validate_scenario_envelope()
            self.envelope_checked = True
            if not self.envelope_valid:
                self.get_logger().error(
                    "Trajectory envelope exceeds configured pool bounds; target disabled"
                )
        if not self.envelope_valid:
            return False
        minimum = np.asarray(self.get_parameter("pool_min_xyz").value, dtype=np.float64)
        maximum = np.asarray(self.get_parameter("pool_max_xyz").value, dtype=np.float64)
        point = np.asarray(position, dtype=np.float64)
        return bool(
            minimum.shape == (3,)
            and maximum.shape == (3,)
            and np.all(minimum < maximum)
            and np.all(point >= minimum)
            and np.all(point <= maximum)
        )

    def validate_scenario_envelope(self):
        self.envelope_rejection_reason = ""
        trajectory_type = str(self.get_parameter("trajectory_type").value).lower()
        control_mode = str(self.get_parameter("control_mode").value).lower()
        if (
            trajectory_type in {"hold", *AUTOMATIC_TRAJECTORY_TYPES}
            and self.motion_start_position is None
        ):
            return False
        if (
            self.motion_start_position is None
            and self.initial_position is None
            and bool(self.get_parameter("relative_to_initial_pose").value)
        ):
            return False
        minimum = np.asarray(self.get_parameter("pool_min_xyz").value, dtype=np.float64)
        maximum = np.asarray(self.get_parameter("pool_max_xyz").value, dtype=np.float64)
        if (
            minimum.shape != (3,)
            or maximum.shape != (3,)
            or not np.all(minimum < maximum)
        ):
            return False
        if not bool(self.get_parameter("trajectory_limits_configured").value):
            return False
        maximum_linear_speed = float(
            self.get_parameter("max_linear_speed_mps").value
        )
        maximum_angular_speed = float(
            self.get_parameter("max_angular_speed_rps").value
        )
        attitude_min = np.radians(
            np.asarray(
                self.get_parameter("attitude_min_rpy_deg").value,
                dtype=np.float64,
            )
        )
        attitude_max = np.radians(
            np.asarray(
                self.get_parameter("attitude_max_rpy_deg").value,
                dtype=np.float64,
            )
        )
        if (
            maximum_linear_speed <= 0.0
            or maximum_angular_speed <= 0.0
            or attitude_min.shape != (3,)
            or attitude_max.shape != (3,)
            or not np.all(attitude_min < attitude_max)
        ):
            return False
        if trajectory_type == "hold" and control_mode in {
            "station_hold",
            "station_hold_fast",
            "rl_policy",
        }:
            point = np.asarray(self.motion_start_position, dtype=np.float64)
            quaternion = np.asarray(self.motion_start_quaternion, dtype=np.float64)
            target = point.copy()
            if target.shape == (3,) and control_mode == "station_hold":
                target[2] = float(self.get_parameter("center_z").value)
            return bool(
                point.shape == (3,)
                and target.shape == (3,)
                and quaternion.shape == (4,)
                and np.all(np.isfinite(point))
                and np.all(np.isfinite(target))
                and np.all(np.isfinite(quaternion))
                and np.linalg.norm(quaternion) > 1e-9
                and np.all(point >= minimum)
                and np.all(point <= maximum)
                and np.all(target >= minimum)
                and np.all(target <= maximum)
            )
        if trajectory_type == "hold" and control_mode == "altitude_hold":
            point = np.asarray(self.motion_start_position, dtype=np.float64)
            quaternion = np.asarray(self.motion_start_quaternion, dtype=np.float64)
            angular_velocity = (
                None
                if self.latest_angular_velocity is None
                else np.asarray(self.latest_angular_velocity, dtype=np.float64)
            )
            target = point.copy()
            if target.shape == (3,):
                target[2] = float(self.get_parameter("center_z").value)
            if (
                point.shape != (3,)
                or quaternion.shape != (4,)
                or angular_velocity is None
                or angular_velocity.shape != (3,)
                or not np.all(np.isfinite(point))
                or not np.all(np.isfinite(target))
                or not np.all(np.isfinite(quaternion))
                or not np.all(np.isfinite(angular_velocity))
                or np.linalg.norm(quaternion) <= 1e-9
            ):
                self.envelope_rejection_reason = (
                    "altitude hold requires a finite current pose, attitude, and angular rate"
                )
                return False
            if (
                np.any(point < minimum)
                or np.any(point > maximum)
                or np.any(target < minimum)
                or np.any(target > maximum)
            ):
                self.envelope_rejection_reason = (
                    "altitude hold start or configured height is outside pool bounds"
                )
                return False
            rpy = self.quaternion_to_rpy(quaternion)
            if np.any(rpy[:2] < attitude_min[:2]) or np.any(
                rpy[:2] > attitude_max[:2]
            ):
                self.envelope_rejection_reason = (
                    "altitude hold start roll/pitch exceeds configured limits"
                )
                return False
            if np.linalg.norm(angular_velocity) > maximum_angular_speed:
                self.envelope_rejection_reason = (
                    "altitude hold start angular rate exceeds configured limit"
                )
                return False
            if abs(float(self.get_parameter("manual_vertical_speed_mps").value)) > (
                maximum_linear_speed
            ):
                self.envelope_rejection_reason = (
                    "altitude hold manual vertical speed exceeds configured limit"
                )
                return False
            # Holding the current heading has zero relative-yaw demand. Global
            # map yaw is intentionally unrestricted; only tilt and rate are
            # safety-bounded for this altitude-only mode.
            return True
        if trajectory_type in AUTOMATIC_TRAJECTORY_TYPES:
            move_duration = max(
                0.1, float(self.get_parameter("move_duration_s").value)
            )
            prelude_duration = max(
                0.0, float(self.get_parameter("hold_before_motion_s").value)
            )
            turn_duration = prelude_duration - move_duration
            if turn_duration < 0.1:
                self.envelope_rejection_reason = (
                    "automatic trajectory prelude must include at least 0.1 s "
                    "for heading alignment after the start approach"
                )
                return False
            for prelude_time_s in np.linspace(0.0, prelude_duration, 201):
                move_time_s = float(
                    np.clip(prelude_time_s, 0.0, move_duration)
                )
                turn_time_s = max(0.0, float(prelude_time_s) - move_duration)
                move_sample = self.sample(
                    "move_to_trajectory_start", float(move_time_s)
                )
                move_point = np.asarray(move_sample.position, dtype=np.float64)
                move_orientation = self.attitude_quaternion(
                    "move_to_trajectory_start", turn_time_s
                )
                move_angular_velocity = self.angular_velocity_at(
                    "move_to_trajectory_start",
                    turn_time_s,
                    1.0
                    / max(
                        20.0,
                        float(self.get_parameter("publish_rate_hz").value),
                    ),
                )
                move_rpy = self.quaternion_to_rpy(move_orientation)
                if (
                    np.any(move_point < minimum)
                    or np.any(move_point > maximum)
                    or np.linalg.norm(move_sample.velocity) > maximum_linear_speed
                    or np.linalg.norm(move_angular_velocity) > maximum_angular_speed
                    or np.any(move_rpy[:2] < attitude_min[:2])
                    or np.any(move_rpy[:2] > attitude_max[:2])
                ):
                    self.envelope_rejection_reason = (
                        "automatic trajectory start approach exceeds configured limits"
                    )
                    return False
        path_speed = max(
            1.0e-9,
            abs(float(self.get_parameter("trajectory_speed_mps").value)),
        )
        if trajectory_type == "spatial_lissajous":
            _phases, cumulative_length = self.lissajous_arc_table(
                abs(float(self.get_parameter("amp_x").value)),
                abs(float(self.get_parameter("amp_y").value)),
                abs(float(self.get_parameter("amp_z").value)),
            )
            path_duration = float(cumulative_length[-1]) / path_speed
        elif trajectory_type == "circle":
            path_duration = (
                2.0
                * math.pi
                * abs(float(self.get_parameter("radius_m").value))
                / path_speed
            )
        elif trajectory_type == "racetrack":
            radius = max(
                1.0e-6, abs(float(self.get_parameter("amp_y").value))
            )
            outer_half_length = max(
                radius, abs(float(self.get_parameter("amp_x").value))
            )
            lap_length = (
                4.0 * (outer_half_length - radius) + 2.0 * math.pi * radius
            )
            path_duration = lap_length / path_speed
        else:
            path_duration = (
                2.0
                * math.pi
                * abs(float(self.get_parameter("amp_x").value))
                / path_speed
            )
        duration = max(
            path_duration
            + max(0.0, float(self.get_parameter("trajectory_ramp_s").value)),
            float(self.get_parameter("move_duration_s").value) + 1.0,
        )
        for time_s in np.linspace(0.0, duration, 361):
            sample = self.sample(trajectory_type, float(time_s))
            point = np.asarray(sample.position, dtype=np.float64)
            orientation = self.attitude_quaternion(trajectory_type, float(time_s))
            angular_velocity = self.angular_velocity_at(
                trajectory_type,
                float(time_s),
                1.0 / max(20.0, float(self.get_parameter("publish_rate_hz").value)),
            )
            rpy = self.quaternion_to_rpy(orientation)
            attitude_axes = (
                slice(0, 2)
                if trajectory_type in AUTOMATIC_TRAJECTORY_TYPES
                else slice(None)
            )
            if (
                np.any(point < minimum)
                or np.any(point > maximum)
                or np.linalg.norm(sample.velocity) > maximum_linear_speed
                or np.linalg.norm(angular_velocity) > maximum_angular_speed
                or np.any(rpy[attitude_axes] < attitude_min[attitude_axes])
                or np.any(rpy[attitude_axes] > attitude_max[attitude_axes])
            ):
                return False
        return True

    @staticmethod
    def level_heading_quaternion(quaternion):
        """Keep measured heading while commanding zero roll and pitch."""

        yaw = TrajectoryCommandNode.quaternion_to_rpy(quaternion)[2]
        return TrajectoryCommandNode.rpy_quaternion(0.0, 0.0, float(yaw))

    @staticmethod
    def quaternion_to_rpy(quaternion):
        w, x, y, z = quaternion
        return np.asarray(
            [
                math.atan2(
                    2.0 * (w * x + y * z),
                    1.0 - 2.0 * (x * x + y * y),
                ),
                math.asin(
                    float(np.clip(2.0 * (w * y - z * x), -1.0, 1.0))
                ),
                math.atan2(
                    2.0 * (w * z + x * y),
                    1.0 - 2.0 * (y * y + z * z),
                ),
            ],
            dtype=np.float64,
        )

    @staticmethod
    def rpy_quaternion(roll, pitch, yaw):
        cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
        cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
        cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
        return np.asarray(
            [
                cr * cp * cy + sr * sp * sy,
                sr * cp * cy - cr * sp * sy,
                cr * sp * cy + sr * cp * sy,
                cr * cp * sy - sr * sp * cy,
            ],
            dtype=np.float64,
        )

    @staticmethod
    def quaternion_multiply(left, right):
        lw, lx, ly, lz = left
        rw, rx, ry, rz = right
        result = np.asarray(
            [
                lw * rw - lx * rx - ly * ry - lz * rz,
                lw * rx + lx * rw + ly * rz - lz * ry,
                lw * ry - lx * rz + ly * rw + lz * rx,
                lw * rz + lx * ry - ly * rx + lz * rw,
            ],
            dtype=np.float64,
        )
        return result / np.linalg.norm(result)

    @staticmethod
    def quaternion_slerp(start, target, fraction):
        start_q = np.asarray(start, dtype=np.float64)
        target_q = np.asarray(target, dtype=np.float64)
        start_q /= np.linalg.norm(start_q)
        target_q /= np.linalg.norm(target_q)
        dot = float(np.dot(start_q, target_q))
        if dot < 0.0:
            target_q = -target_q
            dot = -dot
        dot = float(np.clip(dot, -1.0, 1.0))
        weight = float(np.clip(fraction, 0.0, 1.0))
        if dot > 0.9995:
            result = start_q + weight * (target_q - start_q)
            return result / np.linalg.norm(result)
        angle = math.acos(dot)
        sine = math.sin(angle)
        result = (
            math.sin((1.0 - weight) * angle) / sine * start_q
            + math.sin(weight * angle) / sine * target_q
        )
        return result / np.linalg.norm(result)

    @staticmethod
    def rotation_matrix(quaternion):
        w, x, y, z = quaternion
        return np.asarray(
            [
                [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
            ],
            dtype=np.float64,
        )

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryCommandNode()
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
