#!/usr/bin/env python3
"""Feed deterministic synthetic motion through the 60 Hz edge estimator."""

from __future__ import annotations

import argparse
import math
import statistics
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu

from eup_interfaces.msg import BodyState


def stamp_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


class EstimatorRateCheck(Node):
    def __init__(self, duration_s: float):
        super().__init__("estimator_rate_check")
        self.duration_s = max(3.0, float(duration_s))
        self.started_ns = self.get_clock().now().nanoseconds
        self.odom_publisher = self.create_publisher(
            Odometry, "/localization/aligned_vio_odom", 20
        )
        self.imu_publisher = self.create_publisher(Imu, "/sensors/external_imu", 50)
        self.create_subscription(
            Odometry, "/localization/fused_odom", self.on_fused_odometry, 100
        )
        self.create_subscription(BodyState, "/robot/body_state", self.on_body_state, 100)
        self.create_timer(1.0 / 30.0, self.publish_visual_anchor)
        self.create_timer(1.0 / 60.0, self.publish_imu)
        self.fused_arrival_ns: list[int] = []
        self.fused_stamp_ns: list[int] = []
        self.body_arrival_ns: list[int] = []
        self.body_stamp_ns: list[int] = []
        self.body_velocity_x: list[float] = []

    def publish_visual_anchor(self):
        now = self.get_clock().now()
        elapsed_s = (now.nanoseconds - self.started_ns) * 1e-9
        message = Odometry()
        message.header.stamp = now.to_msg()
        message.header.frame_id = "map"
        message.child_frame_id = "base_link"
        message.pose.pose.position.x = 0.25 * elapsed_s
        message.pose.pose.orientation.w = 1.0
        message.twist.twist.linear.x = 0.25
        for index in (0, 7, 14):
            message.pose.covariance[index] = 0.0025
            message.twist.covariance[index] = 0.0025
        for index in (21, 28, 35):
            message.pose.covariance[index] = 0.001
            message.twist.covariance[index] = 0.001
        self.odom_publisher.publish(message)

    def publish_imu(self):
        message = Imu()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "base_link"
        message.orientation_covariance[0] = -1.0
        message.linear_acceleration_covariance[0] = -1.0
        for index in (0, 4, 8):
            message.angular_velocity_covariance[index] = 0.0001
        self.imu_publisher.publish(message)

    def on_fused_odometry(self, message: Odometry):
        self.fused_arrival_ns.append(self.get_clock().now().nanoseconds)
        self.fused_stamp_ns.append(stamp_ns(message.header.stamp))

    def on_body_state(self, message: BodyState):
        self.body_arrival_ns.append(self.get_clock().now().nanoseconds)
        self.body_stamp_ns.append(stamp_ns(message.header.stamp))
        self.body_velocity_x.append(float(message.twist.linear.x))

    @staticmethod
    def rate_hz(times_ns: list[int]) -> float:
        if len(times_ns) < 2:
            return 0.0
        elapsed_s = (times_ns[-1] - times_ns[0]) * 1e-9
        return (len(times_ns) - 1) / elapsed_s if elapsed_s > 0.0 else 0.0

    @staticmethod
    def strictly_increasing(values: list[int]) -> bool:
        return all(current > previous for previous, current in zip(values, values[1:]))

    def result(self) -> tuple[bool, str]:
        fused_rate = self.rate_hz(self.fused_arrival_ns)
        body_rate = self.rate_hz(self.body_arrival_ns)
        velocity = statistics.median(self.body_velocity_x[-120:]) if self.body_velocity_x else math.nan
        success = (
            len(self.fused_arrival_ns) >= 120
            and len(self.body_arrival_ns) >= 120
            and 57.0 <= fused_rate <= 63.0
            and 57.0 <= body_rate <= 63.0
            and self.strictly_increasing(self.fused_stamp_ns)
            and self.strictly_increasing(self.body_stamp_ns)
            and math.isfinite(velocity)
            and abs(velocity - 0.25) <= 0.03
        )
        summary = (
            f"fused_count={len(self.fused_arrival_ns)} fused_rate_hz={fused_rate:.3f} "
            f"body_count={len(self.body_arrival_ns)} body_rate_hz={body_rate:.3f} "
            f"body_velocity_x_mps={velocity:.4f} "
            f"fused_stamps_monotonic={self.strictly_increasing(self.fused_stamp_ns)} "
            f"body_stamps_monotonic={self.strictly_increasing(self.body_stamp_ns)}"
        )
        return success, summary


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=6.0)
    arguments = parser.parse_args()
    rclpy.init()
    node = EstimatorRateCheck(arguments.duration)
    deadline = time.monotonic() + node.duration_s
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        success, summary = node.result()
        print(summary)
        return 0 if success else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
