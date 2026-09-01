"""Trajectory tracking diagnostics node.

The simulator, policy, UI, and logger should all observe the same ROS contract.
This node keeps tracking-error math in runtime instead of embedding it in the
hardware backend or browser code.
"""

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu

from robotcore_interfaces.msg import BodyState, TrackingStatus, TrajectoryTarget


class TrackingMonitorNode(Node):
    """Publish position, velocity, speed, and acceleration tracking diagnostics."""

    def __init__(self):
        super().__init__("tracking_monitor_node")
        sensor_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.BEST_EFFORT
        )
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("max_input_age_s", 0.15)
        self.declare_parameter("imu_topic", "/sensors/external_imu")

        self.last_body: BodyState | None = None
        self.last_body_ns: int | None = None
        self.last_imu: Imu | None = None
        self.last_imu_ns: int | None = None
        self.last_target: TrajectoryTarget | None = None
        self.last_target_ns: int | None = None
        self.previous_velocity_body: tuple[float, float, float] | None = None
        self.previous_velocity_ns: int | None = None
        self.filtered_acceleration_body = (0.0, 0.0, 0.0)

        self.publisher = self.create_publisher(TrackingStatus, "/runtime/tracking_status", 1)
        self.create_subscription(BodyState, "/robot/body_state", self.on_body_state, 1)
        self.create_subscription(
            Imu,
            str(self.get_parameter("imu_topic").value),
            self.on_imu,
            sensor_qos,
        )
        self.create_subscription(
            TrajectoryTarget,
            "/runtime/trajectory_target",
            self.on_trajectory_target,
            1,
        )

        rate = float(self.get_parameter("publish_rate_hz").value)
        self.timer = self.create_timer(1.0 / max(rate, 0.1), self.tick)

    def on_body_state(self, msg: BodyState):
        self.last_body = msg
        self.last_body_ns = self.get_clock().now().nanoseconds

    def on_imu(self, msg: Imu):
        quaternion = (
            float(msg.orientation.w),
            float(msg.orientation.x),
            float(msg.orientation.y),
            float(msg.orientation.z),
        )
        norm = math.sqrt(sum(value * value for value in quaternion))
        if (
            msg.header.frame_id != "base_link"
            or msg.orientation_covariance[0] < 0.0
            or not math.isfinite(norm)
            or norm <= 1e-9
        ):
            return
        self.last_imu = msg
        self.last_imu_ns = self.get_clock().now().nanoseconds

    def on_trajectory_target(self, msg: TrajectoryTarget):
        self.last_target = msg
        self.last_target_ns = self.get_clock().now().nanoseconds

    def tick(self):
        """Publish one status sample when both inputs are available."""

        if self.last_body is None or self.last_imu is None or self.last_target is None:
            return

        now = self.get_clock().now()
        now_ns = now.nanoseconds
        localization_valid = (
            self.last_body.state_valid or self.last_body.position_estimated
        )
        valid = (
            self.inputs_are_fresh(now_ns)
            and localization_valid
            and self.last_body.linear_velocity_valid
            and self.last_target.valid
        )

        body = self.last_body
        imu = self.last_imu
        target = self.last_target
        actual_pos = (
            float(body.pose.position.x),
            float(body.pose.position.y),
            float(body.pose.position.z),
        )
        target_pos = (
            float(target.target_pose.position.x),
            float(target.target_pose.position.y),
            float(target.target_pose.position.z),
        )
        actual_velocity_body = (
            float(body.twist.linear.x),
            float(body.twist.linear.y),
            float(body.twist.linear.z),
        )

        imu_quat_w = (
            float(imu.orientation.w),
            float(imu.orientation.x),
            float(imu.orientation.y),
            float(imu.orientation.z),
        )
        localization_quat_w = (
            float(body.pose.orientation.w),
            float(body.pose.orientation.x),
            float(body.pose.orientation.y),
            float(body.pose.orientation.z),
        )
        imu_rpy = self.quaternion_to_rpy(imu_quat_w)
        map_yaw = self.quaternion_to_rpy(localization_quat_w)[2]
        control_attitude_w = self.rpy_quaternion(
            imu_rpy[0], imu_rpy[1], map_yaw
        )
        map_to_body = self.quat_conjugate_wxyz(localization_quat_w)
        attitude_to_body = self.quat_conjugate_wxyz(control_attitude_w)
        target_velocity_body = self.quat_apply_wxyz(
            map_to_body,
            (
                float(target.target_twist.linear.x),
                float(target.target_twist.linear.y),
                float(target.target_twist.linear.z),
            ),
        )
        target_acceleration_body = self.quat_apply_wxyz(
            map_to_body,
            (
                float(target.target_accel.linear.x),
                float(target.target_accel.linear.y),
                float(target.target_accel.linear.z),
            ),
        )
        actual_acceleration_body = self.estimate_actual_acceleration(actual_velocity_body, now_ns)
        position_error_world = tuple(
            target_pos[index] - actual_pos[index] for index in range(3)
        )
        position_error_body = self.quat_apply_wxyz(map_to_body, position_error_world)
        target_angular_velocity_body = self.quat_apply_wxyz(
            attitude_to_body,
            (
                float(target.target_twist.angular.x),
                float(target.target_twist.angular.y),
                float(target.target_twist.angular.z),
            ),
        )
        actual_angular_velocity_body = (
            float(imu.angular_velocity.x),
            float(imu.angular_velocity.y),
            float(imu.angular_velocity.z),
        )
        target_quat_w = (
            float(target.target_pose.orientation.w),
            float(target.target_pose.orientation.x),
            float(target.target_pose.orientation.y),
            float(target.target_pose.orientation.z),
        )
        orientation_error_body = self.quaternion_error_body(
            control_attitude_w, target_quat_w
        )

        status = TrackingStatus()
        status.header.stamp = now.to_msg()
        status.header.frame_id = body.header.frame_id or target.header.frame_id or "map"
        status.trajectory_type = str(target.trajectory_type)
        status.control_mode = str(target.control_mode)
        status.valid = bool(valid)
        status.time_s = float(target.time_s)
        status.target_position.x, status.target_position.y, status.target_position.z = target_pos
        status.actual_position.x, status.actual_position.y, status.actual_position.z = actual_pos
        status.target_orientation = target.target_pose.orientation
        (
            status.actual_orientation.w,
            status.actual_orientation.x,
            status.actual_orientation.y,
            status.actual_orientation.z,
        ) = control_attitude_w
        self.assign_vector(status.target_velocity_body, target_velocity_body)
        self.assign_vector(status.actual_velocity_body, actual_velocity_body)
        self.assign_vector(status.target_acceleration_body, target_acceleration_body)
        self.assign_vector(status.actual_acceleration_body, actual_acceleration_body)
        self.assign_vector(status.target_angular_velocity_body, target_angular_velocity_body)
        self.assign_vector(status.actual_angular_velocity_body, actual_angular_velocity_body)
        self.assign_vector(status.position_error_body, position_error_body)
        self.assign_vector(status.orientation_error_body, orientation_error_body)
        status.position_error_m = self.vector_norm(position_error_world)
        status.orientation_error_rad = self.vector_norm(orientation_error_body)
        status.velocity_error_mps = self.vector_norm(
            [target_velocity_body[index] - actual_velocity_body[index] for index in range(3)]
        )
        status.angular_velocity_error_rps = self.vector_norm(
            [
                target_angular_velocity_body[index] - actual_angular_velocity_body[index]
                for index in range(3)
            ]
        )
        status.speed_mps = self.vector_norm(actual_velocity_body)
        status.acceleration_mps2 = self.vector_norm(actual_acceleration_body)
        self.publisher.publish(status)

    def inputs_are_fresh(self, now_ns: int) -> bool:
        """Reject stale inputs so the UI can distinguish live tracking from replay gaps."""

        max_age_ns = int(float(self.get_parameter("max_input_age_s").value) * 1e9)
        if (
            self.last_body_ns is None
            or self.last_imu_ns is None
            or self.last_target_ns is None
        ):
            return False
        return (
            (now_ns - self.last_body_ns) <= max_age_ns
            and (now_ns - self.last_imu_ns) <= max_age_ns
            and (now_ns - self.last_target_ns) <= max_age_ns
        )

    def estimate_actual_acceleration(self, velocity_body: tuple[float, float, float], now_ns: int):
        """Estimate actual body-frame linear acceleration from successive body states."""

        if self.previous_velocity_body is None or self.previous_velocity_ns is None:
            self.previous_velocity_body = velocity_body
            self.previous_velocity_ns = now_ns
            return self.filtered_acceleration_body

        dt = (now_ns - self.previous_velocity_ns) * 1e-9
        if dt <= 1e-4:
            return self.filtered_acceleration_body

        raw = tuple(
            (velocity_body[index] - self.previous_velocity_body[index]) / dt
            for index in range(3)
        )
        # A small first-order filter keeps derivative noise readable in the UI.
        alpha = 0.35
        self.filtered_acceleration_body = tuple(
            alpha * raw[index] + (1.0 - alpha) * self.filtered_acceleration_body[index]
            for index in range(3)
        )
        self.previous_velocity_body = velocity_body
        self.previous_velocity_ns = now_ns
        return self.filtered_acceleration_body

    @staticmethod
    def assign_vector(vector_msg, values):
        vector_msg.x = float(values[0])
        vector_msg.y = float(values[1])
        vector_msg.z = float(values[2])

    @staticmethod
    def vector_norm(values) -> float:
        return math.sqrt(sum(float(value) ** 2 for value in values))

    @staticmethod
    def quat_conjugate_wxyz(quat):
        return (float(quat[0]), -float(quat[1]), -float(quat[2]), -float(quat[3]))

    @staticmethod
    def quat_apply_wxyz(quat, vector):
        """Rotate a vector by a wxyz quaternion using the IsaacLab convention."""

        w, x, y, z = (float(value) for value in quat)
        vx, vy, vz = (float(value) for value in vector)
        tx = 2.0 * (y * vz - z * vy)
        ty = 2.0 * (z * vx - x * vz)
        tz = 2.0 * (x * vy - y * vx)
        return (
            vx + w * tx + (y * tz - z * ty),
            vy + w * ty + (z * tx - x * tz),
            vz + w * tz + (x * ty - y * tx),
        )

    @staticmethod
    def quaternion_to_rpy(quaternion):
        w, x, y, z = (float(value) for value in quaternion)
        norm = math.sqrt(w * w + x * x + y * y + z * z)
        if norm <= 1e-9:
            raise ValueError("quaternion norm is zero")
        w, x, y, z = (value / norm for value in (w, x, y, z))
        return (
            math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y)),
            math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x)))),
            math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)),
        )

    @staticmethod
    def rpy_quaternion(roll, pitch, yaw):
        cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
        cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
        cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
        return (
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
        )

    @classmethod
    def quaternion_error_body(cls, current, target):
        """Shortest target orientation error as a body-frame rotation vector."""

        current_conjugate = cls.quat_conjugate_wxyz(current)
        cw, cx, cy, cz = current_conjugate
        tw, tx, ty, tz = (float(value) for value in target)
        relative = (
            cw * tw - cx * tx - cy * ty - cz * tz,
            cw * tx + cx * tw + cy * tz - cz * ty,
            cw * ty - cx * tz + cy * tw + cz * tx,
            cw * tz + cx * ty - cy * tx + cz * tw,
        )
        norm = math.sqrt(sum(value * value for value in relative))
        if norm <= 1e-9:
            return (0.0, 0.0, 0.0)
        relative = tuple(value / norm for value in relative)
        if relative[0] < 0.0:
            relative = tuple(-value for value in relative)
        vector_norm = math.sqrt(sum(value * value for value in relative[1:]))
        if vector_norm <= 1e-10:
            return tuple(2.0 * value for value in relative[1:])
        angle = 2.0 * math.atan2(vector_norm, max(0.0, relative[0]))
        return tuple(value * angle / vector_norm for value in relative[1:])


def main(args=None):
    rclpy.init(args=args)
    node = TrackingMonitorNode()
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
