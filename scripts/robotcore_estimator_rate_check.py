#!/usr/bin/env python3
"""Verify 60 Hz BodyState output while AprilTag is absent and VIO remains available.

Never run this publisher in the production ROS domain. The probe publishes
only the production localisation inputs: ZED VIO/status and AprilTag.
"""

from __future__ import annotations

import argparse
import math
import statistics
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from zed_msgs.msg import PosTrackStatus

from robotcore_interfaces.msg import AprilTagPoseEstimate, BodyState


def stamp_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


class EstimatorRateCheck(Node):
    def __init__(self, duration_s: float):
        super().__init__("estimator_rate_check")
        self.duration_s = max(3.0, float(duration_s))
        self.started_ns = self.get_clock().now().nanoseconds
        self.vio_publisher = self.create_publisher(
            Odometry, "/zedx/zed_node/odom", 10
        )
        self.zed_status_publisher = self.create_publisher(
            PosTrackStatus, "/zedx/zed_node/pose/status", 10
        )
        self.tag_publisher = self.create_publisher(
            AprilTagPoseEstimate, "/localization/apriltag_pose", 10
        )
        self.create_subscription(BodyState, "/robot/body_state", self.on_body_state, 100)
        self.create_timer(1.0 / 30.0, self.publish_zed_status)
        self.create_timer(1.0 / 30.0, self.publish_vio)
        self.create_timer(1.0 / 30.0, self.publish_tag_anchor)
        self.body_arrival_ns: list[int] = []
        self.body_stamp_ns: list[int] = []
        self.body_velocity_x: list[float] = []
        self.body_velocity_valid: list[bool] = []
        self.body_frame_ids: list[str] = []
        self.body_tag_residual_x: list[float] = []
        self.body_state_valid: list[bool] = []
        self.body_position_estimated: list[bool] = []
        self.outlier_sent = False

    def publish_zed_status(self):
        """Mark synthetic VIO tracking as valid."""

        message = PosTrackStatus()
        message.odometry_status = PosTrackStatus.OK
        self.zed_status_publisher.publish(message)

    def publish_vio(self):
        """Publish the continuous local pose/velocity source used after Tag loss."""

        now = self.get_clock().now()
        message = Odometry()
        measurement_ns = now.nanoseconds - 50_000_000
        measurement_elapsed_s = (measurement_ns - self.started_ns) * 1e-9
        message.header.stamp = Time(nanoseconds=measurement_ns).to_msg()
        message.header.frame_id = "odom"
        message.child_frame_id = "base_link"
        message.pose.pose.position.x = 0.25 * measurement_elapsed_s
        message.pose.pose.orientation.w = 1.0
        message.twist.twist.linear.x = 0.25
        for index in (0, 7, 14):
            message.pose.covariance[index] = 0.0025
            message.twist.covariance[index] = 0.0009
        for index in (21, 28, 35):
            message.pose.covariance[index] = 0.001
            message.twist.covariance[index] = 0.01
        self.vio_publisher.publish(message)

    def publish_tag_anchor(self):
        """Publish the absolute pose used to estimate map-to-odom alignment."""

        now = self.get_clock().now()
        elapsed_s = (now.nanoseconds - self.started_ns) * 1e-9
        # Establish map->odom, then deliberately remove the absolute source.
        if elapsed_s > 2.0:
            return
        message = AprilTagPoseEstimate()
        measurement_ns = now.nanoseconds - 60_000_000
        measurement_elapsed_s = (measurement_ns - self.started_ns) * 1e-9
        message.header.stamp = Time(nanoseconds=measurement_ns).to_msg()
        message.header.frame_id = "map"
        message.map_generation = 1
        message.pose_valid = True
        message.pose.pose.position.x = 0.25 * measurement_elapsed_s
        # One isolated, internally consistent but globally impossible Tag pose
        # must be rejected by the covariance-aware NIS gate.
        if elapsed_s >= 1.0 and not self.outlier_sent:
            message.pose.pose.position.x += 1.0
            self.outlier_sent = True
        message.pose.pose.orientation.w = 1.0
        for index in (0, 7, 14):
            message.pose.covariance[index] = 0.0025
        for index in (21, 28, 35):
            message.pose.covariance[index] = 0.001
        self.tag_publisher.publish(message)

    def on_body_state(self, message: BodyState):
        self.body_arrival_ns.append(self.get_clock().now().nanoseconds)
        measurement_stamp_ns = stamp_ns(message.header.stamp)
        self.body_stamp_ns.append(measurement_stamp_ns)
        self.body_velocity_x.append(float(message.twist.linear.x))
        self.body_velocity_valid.append(bool(message.linear_velocity_valid))
        self.body_frame_ids.append(message.header.frame_id)
        elapsed_s = (measurement_stamp_ns - self.started_ns) * 1e-9
        self.body_tag_residual_x.append(
            float(message.pose.position.x) - 0.25 * elapsed_s
        )
        self.body_state_valid.append(bool(message.state_valid))
        self.body_position_estimated.append(bool(message.position_estimated))

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
        body_rate = self.rate_hz(self.body_arrival_ns)
        body_min_delta, body_nonmonotonic = self.stamp_delta_summary(
            self.body_stamp_ns
        )
        velocity = statistics.median(self.body_velocity_x[-120:]) if self.body_velocity_x else math.nan
        tag_position_error = (
            statistics.median(self.body_tag_residual_x[-120:])
            if self.body_tag_residual_x
            else math.nan
        )
        maximum_tag_position_error = (
            max(abs(error) for error in self.body_tag_residual_x)
            if self.body_tag_residual_x
            else math.nan
        )
        map_output = bool(self.body_frame_ids) and all(
            frame == "map" for frame in self.body_frame_ids[-120:]
        )
        tag_lost_vio_valid = (
            bool(self.body_position_estimated)
            and all(self.body_position_estimated[-120:])
            and not any(self.body_state_valid[-120:])
            and all(self.body_velocity_valid[-120:])
        )
        success = (
            len(self.body_arrival_ns) >= 120
            and 57.0 <= body_rate <= 63.0
            and self.strictly_increasing(self.body_stamp_ns)
            and math.isfinite(velocity)
            and abs(velocity - 0.25) <= 0.03
            and map_output
            and tag_lost_vio_valid
            and math.isfinite(tag_position_error)
            and abs(tag_position_error) <= 0.05
            and math.isfinite(maximum_tag_position_error)
            and maximum_tag_position_error <= 0.10
            and self.outlier_sent
        )
        summary = (
            f"body_count={len(self.body_arrival_ns)} body_state_rate_hz={body_rate:.3f} "
            f"body_velocity_x_mps={velocity:.4f} "
            f"tag_position_error_m={tag_position_error:.4f} "
            f"maximum_tag_position_error_m={maximum_tag_position_error:.4f} "
            f"outlier_sent={self.outlier_sent} "
            f"map_output={map_output} tag_lost_vio_valid={tag_lost_vio_valid} "
            f"body_stamps_monotonic={self.strictly_increasing(self.body_stamp_ns)} "
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
