"""Trajectory tracking diagnostics node.

The simulator, policy, UI, and logger should all observe the same ROS contract.
This node keeps tracking-error math in runtime instead of embedding it in the
MuJoCo backend or browser code.
"""

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node

from eup_interfaces.msg import BodyState, TrackingStatus, TrajectoryTarget


class TrackingMonitorNode(Node):
    """Publish position, velocity, speed, and acceleration tracking diagnostics."""

    def __init__(self):
        super().__init__("tracking_monitor_node")
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("max_input_age_s", 2.0)

        self.last_body: BodyState | None = None
        self.last_body_ns: int | None = None
        self.last_target: TrajectoryTarget | None = None
        self.last_target_ns: int | None = None
        self.previous_velocity_body: tuple[float, float, float] | None = None
        self.previous_velocity_ns: int | None = None
        self.filtered_acceleration_body = (0.0, 0.0, 0.0)

        self.publisher = self.create_publisher(TrackingStatus, "/runtime/tracking_status", 10)
        self.create_subscription(BodyState, "/robot/body_state", self.on_body_state, 10)
        self.create_subscription(
            TrajectoryTarget,
            "/runtime/trajectory_target",
            self.on_trajectory_target,
            10,
        )

        rate = float(self.get_parameter("publish_rate_hz").value)
        self.timer = self.create_timer(1.0 / max(rate, 0.1), self.tick)

    def on_body_state(self, msg: BodyState):
        self.last_body = msg
        self.last_body_ns = self.get_clock().now().nanoseconds

    def on_trajectory_target(self, msg: TrajectoryTarget):
        self.last_target = msg
        self.last_target_ns = self.get_clock().now().nanoseconds

    def tick(self):
        """Publish one status sample when both inputs are available."""

        if self.last_body is None or self.last_target is None:
            return

        now = self.get_clock().now()
        now_ns = now.nanoseconds
        valid = self.inputs_are_fresh(now_ns) and self.last_body.state_valid and self.last_target.valid

        body = self.last_body
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

        root_quat_w = (
            float(body.pose.orientation.w),
            float(body.pose.orientation.x),
            float(body.pose.orientation.y),
            float(body.pose.orientation.z),
        )
        world_to_body = self.quat_conjugate_wxyz(root_quat_w)
        target_velocity_body = self.quat_apply_wxyz(
            world_to_body,
            (
                float(target.target_twist.linear.x),
                float(target.target_twist.linear.y),
                float(target.target_twist.linear.z),
            ),
        )
        target_acceleration_body = self.quat_apply_wxyz(
            world_to_body,
            (
                float(target.target_accel.linear.x),
                float(target.target_accel.linear.y),
                float(target.target_accel.linear.z),
            ),
        )
        actual_acceleration_body = self.estimate_actual_acceleration(actual_velocity_body, now_ns)

        status = TrackingStatus()
        status.header.stamp = now.to_msg()
        status.header.frame_id = body.header.frame_id or target.header.frame_id or "map"
        status.trajectory_type = str(target.trajectory_type)
        status.valid = bool(valid)
        status.time_s = float(target.time_s)
        status.target_position.x, status.target_position.y, status.target_position.z = target_pos
        status.actual_position.x, status.actual_position.y, status.actual_position.z = actual_pos
        self.assign_vector(status.target_velocity_body, target_velocity_body)
        self.assign_vector(status.actual_velocity_body, actual_velocity_body)
        self.assign_vector(status.target_acceleration_body, target_acceleration_body)
        self.assign_vector(status.actual_acceleration_body, actual_acceleration_body)
        status.position_error_m = self.vector_norm(
            [target_pos[index] - actual_pos[index] for index in range(3)]
        )
        status.velocity_error_mps = self.vector_norm(
            [target_velocity_body[index] - actual_velocity_body[index] for index in range(3)]
        )
        status.speed_mps = self.vector_norm(actual_velocity_body)
        status.acceleration_mps2 = self.vector_norm(actual_acceleration_body)
        self.publisher.publish(status)

    def inputs_are_fresh(self, now_ns: int) -> bool:
        """Reject stale inputs so the UI can distinguish live tracking from replay gaps."""

        max_age_ns = int(float(self.get_parameter("max_input_age_s").value) * 1e9)
        if self.last_body_ns is None or self.last_target_ns is None:
            return False
        return (now_ns - self.last_body_ns) <= max_age_ns and (now_ns - self.last_target_ns) <= max_age_ns

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


def main(args=None):
    rclpy.init(args=args)
    node = TrackingMonitorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
