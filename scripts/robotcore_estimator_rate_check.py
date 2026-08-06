#!/usr/bin/env python3
"""Feed deterministic synthetic motion through an isolated 60 Hz estimator.

Never run this publisher in the production ROS domain.  A standalone ESKF
must set ``imu_topic:=/sensors/external_imu`` to match this
probe, exactly as the production launch does.
"""

from __future__ import annotations

import argparse
import math
import statistics
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu

from robotcore_interfaces.msg import AprilTagPoseEstimate, BodyState


def stamp_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


class EstimatorRateCheck(Node):
    def __init__(self, duration_s: float):
        super().__init__("estimator_rate_check")
        self.duration_s = max(3.0, float(duration_s))
        self.started_ns = self.get_clock().now().nanoseconds
        self.odom_publisher = self.create_publisher(
            Odometry, "/localization/zed_odom", 20
        )
        self.imu_publisher = self.create_publisher(
            Imu, "/sensors/external_imu", 50
        )
        self.tag_publisher = self.create_publisher(
            AprilTagPoseEstimate, "/localization/apriltag_pose", 10
        )
        self.create_subscription(
            Odometry, "/localization/fused_odom", self.on_fused_odometry, 100
        )
        self.create_subscription(BodyState, "/robot/body_state", self.on_body_state, 100)
        self.create_timer(1.0 / 30.0, self.publish_visual_anchor)
        self.create_timer(1.0 / 100.0, self.publish_imu)
        self.create_timer(1.0 / 10.0, self.publish_tag_anchor)
        self.fused_arrival_ns: list[int] = []
        self.fused_stamp_ns: list[int] = []
        self.body_arrival_ns: list[int] = []
        self.body_stamp_ns: list[int] = []
        self.body_velocity_x: list[float] = []
        self.fused_frame_ids: list[str] = []
        self.fused_vio_residual_x: list[float] = []
        self.body_state_valid: list[bool] = []

    def publish_visual_anchor(self):
        now = self.get_clock().now()
        elapsed_s = (now.nanoseconds - self.started_ns) * 1e-9
        message = Odometry()
        message.header.stamp = now.to_msg()
        message.header.frame_id = "odom"
        message.child_frame_id = "base_link"
        # Bootstrap map->odom from consistent observations, then introduce a
        # modest absolute correction. The output must move between the VIO and
        # Tag positions; otherwise Tag is only changing a side-channel frame
        # transform instead of updating the ESKF state.
        tag_correction_m = 0.10 if elapsed_s >= 1.0 else 0.0
        message.pose.pose.position.x = 0.25 * elapsed_s + tag_correction_m
        message.pose.pose.orientation.w = 1.0
        message.twist.twist.linear.x = 0.25
        for index in (0, 7, 14):
            message.pose.covariance[index] = 0.0025
            message.twist.covariance[index] = 0.0025
        for index in (21, 28, 35):
            message.pose.covariance[index] = 0.001
            message.twist.covariance[index] = 0.001
        self.odom_publisher.publish(message)

    def publish_tag_anchor(self):
        """Publish a consistent absolute pose so map-to-odom can be confirmed."""

        now = self.get_clock().now()
        elapsed_s = (now.nanoseconds - self.started_ns) * 1e-9
        message = AprilTagPoseEstimate()
        message.header.stamp = now.to_msg()
        message.header.frame_id = "map"
        message.map_generation = 1
        message.pose_valid = True
        message.pose.pose.position.x = 0.25 * elapsed_s
        message.pose.pose.orientation.w = 1.0
        for index in (0, 7, 14):
            message.pose.covariance[index] = 0.0025
        for index in (21, 28, 35):
            message.pose.covariance[index] = 0.001
        self.tag_publisher.publish(message)

    def publish_imu(self):
        message = Imu()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "base_link"
        message.orientation_covariance[0] = -1.0
        # The production ESKF input is specific force: a stationary, level FLU
        # sensor measures +g on Z. A zero vector would synthesize free fall.
        message.linear_acceleration.z = 9.80665
        for index in (0, 4, 8):
            message.angular_velocity_covariance[index] = 0.0001
            message.linear_acceleration_covariance[index] = 0.01
        self.imu_publisher.publish(message)

    def on_fused_odometry(self, message: Odometry):
        self.fused_arrival_ns.append(self.get_clock().now().nanoseconds)
        measurement_stamp_ns = stamp_ns(message.header.stamp)
        self.fused_stamp_ns.append(measurement_stamp_ns)
        self.fused_frame_ids.append(message.header.frame_id)
        elapsed_s = (measurement_stamp_ns - self.started_ns) * 1e-9
        self.fused_vio_residual_x.append(
            float(message.pose.pose.position.x) - 0.25 * elapsed_s
        )

    def on_body_state(self, message: BodyState):
        self.body_arrival_ns.append(self.get_clock().now().nanoseconds)
        self.body_stamp_ns.append(stamp_ns(message.header.stamp))
        self.body_velocity_x.append(float(message.twist.linear.x))
        self.body_state_valid.append(bool(message.state_valid))

    @staticmethod
    def rate_hz(times_ns: list[int]) -> float:
        if len(times_ns) < 2:
            return 0.0
        elapsed_s = (times_ns[-1] - times_ns[0]) * 1e-9
        return (len(times_ns) - 1) / elapsed_s if elapsed_s > 0.0 else 0.0

    @staticmethod
    def strictly_increasing(values: list[int]) -> bool:
        return all(current > previous for previous, current in zip(values, values[1:]))

    @staticmethod
    def stamp_delta_summary(values: list[int]) -> tuple[int, int]:
        deltas = [current - previous for previous, current in zip(values, values[1:])]
        if not deltas:
            return 0, 0
        return min(deltas), sum(delta <= 0 for delta in deltas)

    def result(self) -> tuple[bool, str]:
        fused_rate = self.rate_hz(self.fused_arrival_ns)
        body_rate = self.rate_hz(self.body_arrival_ns)
        fused_min_delta, fused_nonmonotonic = self.stamp_delta_summary(
            self.fused_stamp_ns
        )
        body_min_delta, body_nonmonotonic = self.stamp_delta_summary(
            self.body_stamp_ns
        )
        velocity = statistics.median(self.body_velocity_x[-120:]) if self.body_velocity_x else math.nan
        tag_position_correction = (
            statistics.median(self.fused_vio_residual_x[-120:])
            if self.fused_vio_residual_x
            else math.nan
        )
        map_output = bool(self.fused_frame_ids) and all(
            frame == "map" for frame in self.fused_frame_ids[-120:]
        )
        absolute_valid = bool(self.body_state_valid) and all(self.body_state_valid[-120:])
        success = (
            len(self.fused_arrival_ns) >= 120
            and len(self.body_arrival_ns) >= 120
            and 57.0 <= fused_rate <= 63.0
            and 57.0 <= body_rate <= 63.0
            and self.strictly_increasing(self.fused_stamp_ns)
            and self.strictly_increasing(self.body_stamp_ns)
            and math.isfinite(velocity)
            and abs(velocity - 0.25) <= 0.03
            and map_output
            and absolute_valid
            and math.isfinite(tag_position_correction)
            and 0.003 <= tag_position_correction <= 0.097
        )
        summary = (
            f"fused_count={len(self.fused_arrival_ns)} fused_rate_hz={fused_rate:.3f} "
            f"body_count={len(self.body_arrival_ns)} body_rate_hz={body_rate:.3f} "
            f"body_velocity_x_mps={velocity:.4f} "
            f"tag_position_correction_m={tag_position_correction:.4f} "
            f"map_output={map_output} absolute_valid={absolute_valid} "
            f"fused_stamps_monotonic={self.strictly_increasing(self.fused_stamp_ns)} "
            f"body_stamps_monotonic={self.strictly_increasing(self.body_stamp_ns)} "
            f"fused_min_stamp_delta_ns={fused_min_delta} "
            f"fused_nonmonotonic_count={fused_nonmonotonic} "
            f"body_min_stamp_delta_ns={body_min_delta} "
            f"body_nonmonotonic_count={body_nonmonotonic}"
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
