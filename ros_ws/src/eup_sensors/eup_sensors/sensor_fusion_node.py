"""Publish the aligned Tag/ZED-VIO localisation state in the body-state contract.

AprilTag is the absolute map observation. ZED visual-inertial odometry,
which already fuses the camera IMU, propagates the local odometry between tag
corrections. ``tag_vio_alignment_node`` owns map-to-odom calibration and
never substitutes a raw Tag pose directly into the body-state output.
"""

from __future__ import annotations

import math
from copy import deepcopy

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32

from eup_interfaces.msg import BodyState


def finite(value) -> bool:
    return math.isfinite(float(value))


def quaternion_valid(orientation) -> bool:
    values = (
        float(orientation.w),
        float(orientation.x),
        float(orientation.y),
        float(orientation.z),
    )
    return all(math.isfinite(value) for value in values) and any(abs(value) > 1e-9 for value in values)


class SensorFusionNode(Node):
    """Adapt map-frame Tag-calibrated ZED VIO output to :class:`BodyState`."""

    def __init__(self):
        super().__init__("sensor_fusion_node")
        self.declare_parameter("body_state_topic", "/robot/body_state")
        self.declare_parameter("depth_input_topic", "")
        self.declare_parameter("altitude_input_topic", "")
        self.declare_parameter("body_frame_id", "map")
        self.declare_parameter("localization_pose_topic", "/localization/apriltag_pose")
        # Strict two-Tag poses validate an already aligned VIO estimate but
        # are intentionally isolated from the map-to-odom alignment node.
        self.declare_parameter(
            "degraded_localization_pose_topic",
            "/localization/apriltag_pose_degraded",
        )
        self.declare_parameter("localization_max_age_s", 0.20)
        self.declare_parameter("fused_odometry_topic", "/localization/fused_odom")
        self.declare_parameter("zed_odometry_topic", "/localization/zed_odom")
        self.declare_parameter("zed_odometry_max_age_s", 0.35)
        self.declare_parameter("external_imu_topic", "/sensors/external_imu")
        self.declare_parameter("external_imu_max_age_s", 0.20)
        # This is an operator-status comparison. The alignment node owns the
        # authoritative correction gate.
        self.declare_parameter("tag_vio_disagreement_m", 0.50)

        self.depth_m = math.nan
        self.altitude_m = math.nan
        self.localization_pose: PoseWithCovarianceStamped | None = None
        self.localization_arrival_ns = 0
        self.zed_odometry_arrival_ns = 0
        self.external_imu_arrival_ns = 0
        self.last_status_warning = ""

        self.body_state_topic = str(self.get_parameter("body_state_topic").value)
        self.body_frame_id = str(self.get_parameter("body_frame_id").value)
        self.fused_odometry_topic = str(self.get_parameter("fused_odometry_topic").value)
        self.zed_odometry_topic = str(self.get_parameter("zed_odometry_topic").value)
        if not self.fused_odometry_topic:
            raise ValueError("fused_odometry_topic must be set")

        self.body_pub = self.create_publisher(BodyState, self.body_state_topic, 10)

        localization_topic = str(self.get_parameter("localization_pose_topic").value)
        if localization_topic:
            self.create_subscription(PoseWithCovarianceStamped, localization_topic, self.on_localization_pose, 10)
        degraded_localization_topic = str(
            self.get_parameter("degraded_localization_pose_topic").value
        )
        if degraded_localization_topic:
            self.create_subscription(
                PoseWithCovarianceStamped,
                degraded_localization_topic,
                self.on_localization_pose,
                10,
            )
        self.create_subscription(Odometry, self.fused_odometry_topic, self.on_fused_odometry, 20)
        if self.zed_odometry_topic:
            self.create_subscription(Odometry, self.zed_odometry_topic, self.on_zed_odometry, 20)
        external_imu_topic = str(self.get_parameter("external_imu_topic").value)
        if external_imu_topic:
            self.create_subscription(Imu, external_imu_topic, self.on_external_imu, 50)

        depth_topic = str(self.get_parameter("depth_input_topic").value)
        if depth_topic:
            self.create_subscription(Float32, depth_topic, self.on_depth, 10)
        altitude_topic = str(self.get_parameter("altitude_input_topic").value)
        if altitude_topic:
            self.create_subscription(Float32, altitude_topic, self.on_altitude, 10)

        self.get_logger().info(
            f"Sensor fusion publishing body state from Tag-calibrated ZED VIO odometry "
            f"{self.fused_odometry_topic}"
        )

    def on_depth(self, msg: Float32):
        self.depth_m = float(msg.data)

    def on_altitude(self, msg: Float32):
        self.altitude_m = float(msg.data)

    def on_localization_pose(self, msg: PoseWithCovarianceStamped):
        if not quaternion_valid(msg.pose.pose.orientation):
            return
        self.localization_pose = deepcopy(msg)
        self.localization_arrival_ns = self.get_clock().now().nanoseconds

    def on_zed_odometry(self, msg: Odometry):
        """Record a recent, finite VIO velocity for state provenance."""

        values = (
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z,
        )
        if all(finite(value) for value in values):
            self.zed_odometry_arrival_ns = self.get_clock().now().nanoseconds

    def on_external_imu(self, msg: Imu):
        values = (
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z,
        )
        if all(finite(value) for value in values):
            self.external_imu_arrival_ns = self.get_clock().now().nanoseconds

    def on_fused_odometry(self, odometry: Odometry):
        """Publish the map-aligned VIO estimate without raw Tag replacement."""

        if not quaternion_valid(odometry.pose.pose.orientation):
            return

        body = self.make_body_state_from_fused_odometry(odometry)
        # robot_localization can continue predicting at 60 Hz after an input
        # stops. Keep the output cadence, but never label predicted/stale VIO
        # velocity as a fresh measured velocity.
        body.linear_velocity_valid = (
            body.linear_velocity_valid and self.zed_odometry_is_fresh()
        )
        tag = self.fresh_localization_pose()
        tag_consistent = tag is not None and self.tag_matches_ekf(body, tag)
        tag_rejected = tag is not None and not tag_consistent

        # A stale tag must never invalidate ZED-VIO dead reckoning. A fresh
        # but inconsistent tag is likewise reported as an estimate; its
        # correction is rejected upstream by the map-to-odom alignment gate.
        body.state_valid = tag_consistent
        body.position_estimated = not tag_consistent
        body.localization_source = self.localization_source(
            tag_accepted=tag_consistent,
            tag_rejected=tag_rejected,
        )
        if tag_rejected:
            self.warn_status(
                "fresh AprilTag disagrees with Tag/VIO alignment; publishing the ZED-VIO estimate"
            )
        else:
            self.last_status_warning = ""
        self.body_pub.publish(body)

    def make_body_state_from_fused_odometry(self, odometry: Odometry) -> BodyState:
        """Copy map-frame aligned pose and base-frame VIO twist into BodyState."""

        body = BodyState()
        body.header = deepcopy(odometry.header)
        if not body.header.frame_id:
            body.header.frame_id = self.body_frame_id
        body.pose = deepcopy(odometry.pose.pose)
        body.twist = deepcopy(odometry.twist.twist)
        body.linear_velocity_valid = all(
            finite(value)
            for value in (body.twist.linear.x, body.twist.linear.y, body.twist.linear.z)
        )
        body.depth_m = float(self.depth_m)
        body.altitude_m = float(self.altitude_m)
        return body

    def tag_matches_ekf(self, body: BodyState, tag: PoseWithCovarianceStamped) -> bool:
        maximum_residual = max(0.0, float(self.get_parameter("tag_vio_disagreement_m").value))
        if maximum_residual == 0.0:
            return True
        fused = body.pose.position
        observed = tag.pose.pose.position
        values = (
            float(fused.x),
            float(fused.y),
            float(fused.z),
            float(observed.x),
            float(observed.y),
            float(observed.z),
        )
        if not all(math.isfinite(value) for value in values):
            return False
        residual_m = math.sqrt(
            (values[0] - values[3]) ** 2
            + (values[1] - values[4]) ** 2
            + (values[2] - values[5]) ** 2
        )
        return residual_m <= maximum_residual

    def zed_odometry_is_fresh(self) -> bool:
        if self.zed_odometry_arrival_ns <= 0:
            return False
        maximum_age_ns = int(
            max(0.0, float(self.get_parameter("zed_odometry_max_age_s").value)) * 1e9
        )
        return (
            maximum_age_ns <= 0
            or self.get_clock().now().nanoseconds - self.zed_odometry_arrival_ns <= maximum_age_ns
        )

    def localization_source(self, *, tag_accepted: bool, tag_rejected: bool) -> str:
        vio_available = self.zed_odometry_is_fresh()
        imu_available = self.external_imu_is_fresh()
        suffix = "+External IMU" if imu_available else ""
        if tag_accepted:
            return f"Tag+ZED VIO{suffix}" if vio_available else f"Tag{suffix}"
        if vio_available:
            source = f"ZED VIO{suffix}"
            return f"{source} (Tag rejected)" if tag_rejected else source
        source = "External IMU" if imu_available else "localization unavailable"
        return f"{source} (Tag rejected)" if tag_rejected else source

    def external_imu_is_fresh(self) -> bool:
        if self.external_imu_arrival_ns <= 0:
            return False
        maximum_age_ns = int(
            max(0.0, float(self.get_parameter("external_imu_max_age_s").value)) * 1e9
        )
        return (
            maximum_age_ns <= 0
            or self.get_clock().now().nanoseconds - self.external_imu_arrival_ns
            <= maximum_age_ns
        )

    def fresh_localization_pose(self) -> PoseWithCovarianceStamped | None:
        if self.localization_pose is None:
            return None
        maximum_age_ns = int(
            max(0.0, float(self.get_parameter("localization_max_age_s").value)) * 1e9
        )
        age_ns = self.get_clock().now().nanoseconds - self.localization_arrival_ns
        return self.localization_pose if age_ns <= maximum_age_ns else None

    def warn_status(self, message: str):
        if message != self.last_status_warning:
            self.get_logger().warn(message)
            self.last_status_warning = message


def main(args=None):
    rclpy.init(args=args)
    node = SensorFusionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
