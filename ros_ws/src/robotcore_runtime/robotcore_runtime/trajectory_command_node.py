"""Isaac-compatible trajectory target publisher.

The exported WarpAUV trajectory policy expects target pose and target linear
velocity as part of its 20-D observation.  IsaacLab generated those commands
inside the environment; in RobotCore they live in ROS so hardware, logging, UI, and
future hardware playback can all see the same target stream.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

from robotcore_interfaces.msg import BodyState, ThrusterCommand, TrajectoryTarget


@dataclass
class TrajectorySample:
    """One target sample in world/map coordinates."""

    position: tuple[float, float, float]
    velocity: tuple[float, float, float]
    acceleration: tuple[float, float, float]


class TrajectoryCommandNode(Node):
    """Publishes deterministic trajectory targets matching Isaac AUV eval code."""

    def __init__(self):
        super().__init__("trajectory_command_node")
        self.declare_parameter("trajectory_type", "lissajous")
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("center_x", 0.0)
        self.declare_parameter("center_y", 0.0)
        self.declare_parameter("center_z", 0.9)
        self.declare_parameter("amp_x", 1.5)
        self.declare_parameter("amp_y", 0.75)
        self.declare_parameter("amp_z", 0.4)
        self.declare_parameter("period_s", 16.0)
        self.declare_parameter("radius_min", 0.3)
        self.declare_parameter("radius_max", 1.5)
        self.declare_parameter("chirp_rate", 2.2)
        self.declare_parameter("relative_to_initial_pose", False)
        self.declare_parameter("hold_before_motion_s", 0.0)
        self.declare_parameter("attitude_mode", "fixed_identity")
        self.declare_parameter("roll_amplitude_deg", 5.0)
        self.declare_parameter("pitch_amplitude_deg", 5.0)
        self.declare_parameter("yaw_amplitude_deg", 15.0)
        self.declare_parameter("attitude_period_s", 30.0)
        self.declare_parameter("step_amplitude", 0.1)
        self.declare_parameter("step_time_s", 5.0)
        self.declare_parameter("move_duration_s", 18.0)
        self.declare_parameter("require_pool_bounds", False)
        self.declare_parameter("pool_min_xyz", [0.0, 0.0, 0.0])
        self.declare_parameter("pool_max_xyz", [0.0, 0.0, 0.0])
        self.declare_parameter("trajectory_limits_configured", False)
        self.declare_parameter("max_linear_speed_mps", 0.0)
        self.declare_parameter("manual_vertical_speed_mps", 0.20)
        self.declare_parameter("manual_input_age_s", 0.15)
        self.declare_parameter("max_angular_speed_rps", 0.0)
        self.declare_parameter("attitude_min_rpy_deg", [0.0, 0.0, 0.0])
        self.declare_parameter("attitude_max_rpy_deg", [0.0, 0.0, 0.0])

        self.started_ns = self.get_clock().now().nanoseconds
        self.tracking_started = False
        self.initial_position = None
        self.initial_quaternion = None
        self.latest_position = None
        self.latest_quaternion = None
        self.motion_start_position = None
        self.altitude_hold_z = None
        self.altitude_heave_active = False
        self.manual_command = None
        self.manual_command_ns = None
        self.envelope_checked = False
        self.envelope_valid = False
        self.pub = self.create_publisher(TrajectoryTarget, "/runtime/trajectory_target", 10)
        self.create_subscription(BodyState, "/robot/body_state", self.on_body_state, 10)
        self.create_subscription(
            ThrusterCommand,
            "/control/candidates/manual",
            self.on_manual_command,
            20,
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
            # PID candidate stays neutral until the explicit Start/reset.
            published_trajectory_type = "idle"
            target_valid = self.idle_target_is_valid(sample.position)
            angular_velocity = (0.0, 0.0, 0.0)
            angular_acceleration = (0.0, 0.0, 0.0)
        elif trajectory_type == "altitude_hold":
            sample = self.altitude_hold_sample(now.nanoseconds)
            orientation = tuple(
                self.latest_quaternion
                if self.latest_quaternion is not None
                else self.attitude_quaternion("hold", 0.0)
            )
            published_trajectory_type = trajectory_type
            target_valid = self.target_is_valid(sample.position)
            angular_velocity = (0.0, 0.0, 0.0)
            angular_acceleration = (0.0, 0.0, 0.0)
        elif time_s < hold_s:
            sample_type = "move_to_hold" if trajectory_type == "move_to_hold" else "hold"
            sample = self.sample(sample_type, 0.0)
            orientation = tuple(self.attitude_quaternion("hold", 0.0))
            published_trajectory_type = trajectory_type
            target_valid = self.target_is_valid(sample.position)
            angular_velocity = (0.0, 0.0, 0.0)
            angular_acceleration = (0.0, 0.0, 0.0)
        else:
            sample = self.sample(trajectory_type, effective_time_s)
            orientation, angular_velocity, angular_acceleration = self.sample_attitude(
                trajectory_type, effective_time_s
            )
            published_trajectory_type = trajectory_type
            target_valid = self.target_is_valid(sample.position)

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
        msg.time_s = float(time_s)
        msg.valid = target_valid
        self.pub.publish(msg)

    def trajectory_time_s(self, now_ns):
        """Keep Target time frozen until the explicit Start/reset service."""

        if not self.tracking_started:
            return 0.0
        return max(0.0, (int(now_ns) - self.started_ns) * 1e-9)

    def on_body_state(self, msg: BodyState):
        """Track the latest pose and retain the first trusted relative center."""

        if not (msg.state_valid or msg.position_estimated):
            return
        quaternion = np.asarray(
            [
                msg.pose.orientation.w,
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
            ],
            dtype=np.float64,
        )
        norm = float(np.linalg.norm(quaternion))
        if not np.isfinite(norm) or norm <= 1e-9:
            return
        position = np.asarray(
            [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z],
            dtype=np.float64,
        )
        if not np.all(np.isfinite(position)):
            return
        self.latest_position = position
        self.latest_quaternion = quaternion / norm
        if self.initial_position is None:
            self.initial_position = position.copy()
            self.initial_quaternion = self.latest_quaternion.copy()
            self.envelope_checked = False
            self.get_logger().info("Captured trusted initial pose for relative trajectory")

    def on_manual_command(self, msg: ThrusterCommand):
        self.manual_command = msg
        self.manual_command_ns = self.get_clock().now().nanoseconds

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
        values = np.asarray(self.manual_command.normalized, dtype=np.float64)
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

    def on_reset_scenario(self, _request, response):
        trajectory_type = str(self.get_parameter("trajectory_type").value).lower()
        if trajectory_type in {"move_to_hold", "altitude_hold"} and self.motion_start_position is None:
            response.success = False
            response.message = f"{trajectory_type} requires a validated current body pose"
            return response
        if trajectory_type == "altitude_hold":
            self.altitude_hold_z = float(self.get_parameter("center_z").value)
            self.altitude_heave_active = False
        self.started_ns = self.get_clock().now().nanoseconds
        self.tracking_started = True
        self.envelope_checked = False
        self.envelope_valid = False
        response.success = True
        response.message = "trajectory tracking started at 0.00 s"
        return response

    def on_stop_scenario(self, _request, response):
        self.tracking_started = False
        self.started_ns = self.get_clock().now().nanoseconds
        self.motion_start_position = None
        self.altitude_hold_z = None
        self.altitude_heave_active = False
        response.success = True
        response.message = "trajectory ready; target time held at 0.00 s"
        return response

    def on_validate_scenario(self, _request, response):
        trajectory_type = str(self.get_parameter("trajectory_type").value).lower()
        if trajectory_type in {"move_to_hold", "altitude_hold"}:
            if self.latest_position is None:
                self.envelope_valid = False
                self.envelope_checked = True
                response.success = False
                response.message = f"{trajectory_type} requires a current valid body pose"
                return response
            # Latch exactly the start point used by both validation and reset;
            # later estimator updates cannot change an already-approved path.
            self.motion_start_position = self.latest_position.copy()
        else:
            self.motion_start_position = None
        self.envelope_valid = self.validate_scenario_envelope()
        self.envelope_checked = True
        response.success = bool(self.envelope_valid)
        response.message = (
            "trajectory envelope is inside all configured limits"
            if self.envelope_valid
            else "trajectory envelope or safety limits are invalid"
        )
        return response

    def sample(self, trajectory_type: str, time_s: float) -> TrajectorySample:
        """Evaluate one of the Isaac AUV trajectory families."""

        if (
            bool(self.get_parameter("relative_to_initial_pose").value)
            and self.initial_position is not None
        ):
            center = tuple(float(value) for value in self.initial_position)
        else:
            center = (
                float(self.get_parameter("center_x").value),
                float(self.get_parameter("center_y").value),
                float(self.get_parameter("center_z").value),
            )
        amp_x = float(self.get_parameter("amp_x").value)
        amp_y = float(self.get_parameter("amp_y").value)
        amp_z = float(self.get_parameter("amp_z").value)
        period = max(0.1, float(self.get_parameter("period_s").value))
        omega = 2.0 * math.pi / period
        phase_x = omega * time_s
        phase_y = 2.0 * omega * time_s

        if trajectory_type == "move_to_hold":
            start = self.motion_start_position
            if start is None:
                start = self.latest_position
            if start is None:
                start = np.asarray(center, dtype=np.float64)
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
        if trajectory_type == "altitude_hold":
            start = self.motion_start_position
            if start is None:
                start = self.latest_position
            if start is None:
                start = np.asarray(center, dtype=np.float64)
            return TrajectorySample(
                position=(float(start[0]), float(start[1]), float(center[2])),
                velocity=(0.0, 0.0, 0.0),
                acceleration=(0.0, 0.0, 0.0),
            )
        if trajectory_type.startswith("step_"):
            offset = [0.0, 0.0, 0.0]
            velocity = [0.0, 0.0, 0.0]
            acceleration = [0.0, 0.0, 0.0]
            axis_name = trajectory_type.rsplit("_", 1)[-1]
            if axis_name in {"x", "y", "z"}:
                axis = {"x": 0, "y": 1, "z": 2}[axis_name]
                if time_s >= float(self.get_parameter("step_time_s").value):
                    offset[axis] = float(self.get_parameter("step_amplitude").value)
            offset, velocity, acceleration = tuple(offset), tuple(velocity), tuple(acceleration)
        elif trajectory_type == "hold" or trajectory_type.startswith("step_"):
            offset = (0.0, 0.0, 0.0)
            velocity = (0.0, 0.0, 0.0)
            acceleration = (0.0, 0.0, 0.0)
        elif trajectory_type == "circle":
            offset = (
                amp_x * math.cos(phase_x),
                amp_y * math.sin(phase_x),
                0.0,
            )
            velocity = (
                -amp_x * omega * math.sin(phase_x),
                amp_y * omega * math.cos(phase_x),
                0.0,
            )
            acceleration = (
                -amp_x * omega**2 * math.cos(phase_x),
                -amp_y * omega**2 * math.sin(phase_x),
                0.0,
            )
        elif trajectory_type == "helix":
            offset = (
                amp_x * math.cos(phase_x),
                amp_y * math.sin(phase_x),
                amp_z * math.sin(phase_y),
            )
            velocity = (
                -amp_x * omega * math.sin(phase_x),
                amp_y * omega * math.cos(phase_x),
                2.0 * amp_z * omega * math.cos(phase_y),
            )
            acceleration = (
                -amp_x * omega**2 * math.cos(phase_x),
                -amp_y * omega**2 * math.sin(phase_x),
                -4.0 * amp_z * omega**2 * math.sin(phase_y),
            )
        elif trajectory_type == "spiral":
            offset, velocity, acceleration = self._sample_spiral(time_s, omega)
        elif trajectory_type == "chirp":
            offset, velocity, acceleration = self._sample_chirp(time_s, omega)
        elif trajectory_type == "racetrack":
            offset, velocity, acceleration = self._sample_racetrack(time_s)
        elif trajectory_type in {"sine_x", "sine_y", "sine_z"}:
            axis = {"sine_x": 0, "sine_y": 1, "sine_z": 2}[trajectory_type]
            offset = [0.0, 0.0, 0.0]
            velocity = [0.0, 0.0, 0.0]
            acceleration = [0.0, 0.0, 0.0]
            offset[axis] = amp_x * math.sin(phase_x)
            velocity[axis] = amp_x * omega * math.cos(phase_x)
            acceleration[axis] = -amp_x * omega**2 * math.sin(phase_x)
            offset = tuple(offset)
            velocity = tuple(velocity)
            acceleration = tuple(acceleration)
        elif trajectory_type == "pose_lissajous_6dof":
            offset = (
                amp_x * math.sin(phase_x),
                amp_y * math.sin(phase_y),
                amp_z * math.sin(3.0 * phase_x),
            )
            velocity = (
                amp_x * omega * math.cos(phase_x),
                2.0 * amp_y * omega * math.cos(phase_y),
                3.0 * amp_z * omega * math.cos(3.0 * phase_x),
            )
            acceleration = (
                -amp_x * omega**2 * math.sin(phase_x),
                -4.0 * amp_y * omega**2 * math.sin(phase_y),
                -9.0 * amp_z * omega**2 * math.sin(3.0 * phase_x),
            )
        else:
            # Isaac's default trajectory policy was trained/evaluated on a
            # Lissajous-style figure-eight, so unknown names fall back there.
            offset = (
                amp_x * math.sin(phase_x),
                amp_y * math.sin(phase_y),
                0.0,
            )
            velocity = (
                amp_x * omega * math.cos(phase_x),
                2.0 * amp_y * omega * math.cos(phase_y),
                0.0,
            )
            acceleration = (
                -amp_x * omega**2 * math.sin(phase_x),
                -4.0 * amp_y * omega**2 * math.sin(phase_y),
                0.0,
            )

        position = (
            center[0] + offset[0],
            center[1] + offset[1],
            center[2] + offset[2],
        )
        return TrajectorySample(position=position, velocity=velocity, acceleration=acceleration)

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
        mode = str(self.get_parameter("attitude_mode").value).lower()
        use_initial_attitude = (
            mode == "hold_initial"
            or bool(self.get_parameter("relative_to_initial_pose").value)
        )
        base = (
            self.initial_quaternion
            if use_initial_attitude and self.initial_quaternion is not None
            else np.asarray([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
        )
        rpy = np.zeros(3, dtype=np.float64)
        if trajectory_type in {"step_roll", "step_pitch", "step_yaw"}:
            if time_s >= float(self.get_parameter("step_time_s").value):
                axis = {"step_roll": 0, "step_pitch": 1, "step_yaw": 2}[trajectory_type]
                rpy[axis] = math.radians(
                    float(self.get_parameter("step_amplitude").value)
                )
        elif mode == "six_dof_sine":
            period = max(0.1, float(self.get_parameter("attitude_period_s").value))
            phase = 2.0 * math.pi * time_s / period
            amplitudes = np.radians(
                [
                    float(self.get_parameter("roll_amplitude_deg").value),
                    float(self.get_parameter("pitch_amplitude_deg").value),
                    float(self.get_parameter("yaw_amplitude_deg").value),
                ]
            )
            rpy = amplitudes * np.asarray(
                [math.sin(phase), math.sin(2.0 * phase), math.sin(0.5 * phase)]
            )
        elif mode == "fixed_identity":
            base = np.asarray([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
        offset = self.rpy_quaternion(*rpy)
        return self.quaternion_multiply(base, offset)

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
        if relative and self.initial_position is None:
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
        trajectory_type = str(self.get_parameter("trajectory_type").value).lower()
        if trajectory_type in {"move_to_hold", "altitude_hold"} and self.motion_start_position is None:
            return False
        if self.initial_position is None and bool(
            self.get_parameter("relative_to_initial_pose").value
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
        duration = max(
            float(self.get_parameter("period_s").value),
            2.0 * float(self.get_parameter("attitude_period_s").value),
            float(self.get_parameter("step_time_s").value) + 1.0,
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
            if (
                np.any(point < minimum)
                or np.any(point > maximum)
                or np.linalg.norm(sample.velocity) > maximum_linear_speed
                or np.linalg.norm(angular_velocity) > maximum_angular_speed
                or np.any(rpy < attitude_min)
                or np.any(rpy > attitude_max)
            ):
                return False
        return True

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

    def _sample_spiral(self, time_s: float, omega: float):
        radius_min = float(self.get_parameter("radius_min").value)
        radius_max = float(self.get_parameter("radius_max").value)
        radius_range = radius_max - radius_min
        radial_phase = 0.5 * omega * time_s
        radius = radius_min + 0.5 * radius_range * (1.0 - math.cos(radial_phase))
        radius_dot = 0.25 * radius_range * omega * math.sin(radial_phase)
        radius_ddot = 0.125 * radius_range * omega**2 * math.cos(radial_phase)
        phase = omega * time_s
        offset = (
            radius * math.cos(phase),
            radius * math.sin(phase),
            0.0,
        )
        velocity = (
            radius_dot * math.cos(phase) - radius * omega * math.sin(phase),
            radius_dot * math.sin(phase) + radius * omega * math.cos(phase),
            0.0,
        )
        acceleration = (
            radius_ddot * math.cos(phase)
            - 2.0 * radius_dot * omega * math.sin(phase)
            - radius * omega**2 * math.cos(phase),
            radius_ddot * math.sin(phase)
            + 2.0 * radius_dot * omega * math.cos(phase)
            - radius * omega**2 * math.sin(phase),
            0.0,
        )
        return offset, velocity, acceleration

    def _sample_chirp(self, time_s: float, omega: float):
        amp_x = float(self.get_parameter("amp_x").value)
        amp_y = float(self.get_parameter("amp_y").value)
        chirp_rate = float(self.get_parameter("chirp_rate").value)
        period = max(0.1, float(self.get_parameter("period_s").value))
        w1 = chirp_rate * omega
        chirp_k = (w1 - omega) / period
        phase = omega * time_s + 0.5 * chirp_k * time_s**2
        chirp_omega = omega + chirp_k * time_s
        offset = (
            amp_x * math.sin(phase),
            amp_y * math.sin(2.0 * phase),
            0.0,
        )
        velocity = (
            amp_x * chirp_omega * math.cos(phase),
            2.0 * amp_y * chirp_omega * math.cos(2.0 * phase),
            0.0,
        )
        acceleration = (
            amp_x * (chirp_k * math.cos(phase) - chirp_omega**2 * math.sin(phase)),
            2.0
            * amp_y
            * (chirp_k * math.cos(2.0 * phase) - 2.0 * chirp_omega**2 * math.sin(2.0 * phase)),
            0.0,
        )
        return offset, velocity, acceleration

    def _sample_racetrack(self, time_s: float):
        amp_x = float(self.get_parameter("amp_x").value)
        radius = max(0.05, float(self.get_parameter("amp_y").value))
        period = max(0.1, float(self.get_parameter("period_s").value))
        half_straight = max(0.1, amp_x - radius)
        path_length = 4.0 * half_straight + 2.0 * math.pi * radius
        speed = path_length / period
        s = (speed * time_s) % path_length

        if s < 2.0 * half_straight:
            return (
                (-half_straight + s, radius, 0.0),
                (speed, 0.0, 0.0),
                (0.0, 0.0, 0.0),
            )
        if s < 2.0 * half_straight + math.pi * radius:
            local_s = s - 2.0 * half_straight
            theta = math.pi / 2.0 - local_s / radius
            return (
                half_straight + radius * math.cos(theta),
                radius * math.sin(theta),
                0.0,
            ), (
                speed * math.sin(theta),
                -speed * math.cos(theta),
                0.0,
            ), (
                -(speed**2 / radius) * math.cos(theta),
                -(speed**2 / radius) * math.sin(theta),
                0.0,
            )
        if s < 4.0 * half_straight + math.pi * radius:
            local_s = s - (2.0 * half_straight + math.pi * radius)
            return (
                (half_straight - local_s, -radius, 0.0),
                (-speed, 0.0, 0.0),
                (0.0, 0.0, 0.0),
            )

        local_s = s - (4.0 * half_straight + math.pi * radius)
        theta = -math.pi / 2.0 - local_s / radius
        return (
            -half_straight + radius * math.cos(theta),
            radius * math.sin(theta),
            0.0,
        ), (
            speed * math.sin(theta),
            -speed * math.cos(theta),
            0.0,
        ), (
            -(speed**2 / radius) * math.cos(theta),
            -(speed**2 / radius) * math.sin(theta),
            0.0,
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
