"""Convert ZED camera-frame VIO odometry into the vehicle base_link frame."""

from __future__ import annotations

import math

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from .localization_math import invert_transform, quaternion_xyzw


def rotation_from_quaternion(quaternion) -> np.ndarray:
    """Return the active 3x3 rotation represented by a ROS xyzw quaternion."""

    x, y, z, w = (float(quaternion.x), float(quaternion.y), float(quaternion.z), float(quaternion.w))
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(norm) or norm <= 1e-9:
        raise ValueError("transform quaternion is invalid")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def transform_from_pose(pose) -> np.ndarray:
    """Return the parent-from-child rigid transform carried by an Odometry pose."""

    rotation = rotation_from_quaternion(pose.orientation)
    result = np.eye(4, dtype=np.float64)
    result[:3, :3] = rotation
    result[:3, 3] = [float(pose.position.x), float(pose.position.y), float(pose.position.z)]
    return result


def set_pose_from_transform(pose, transform: np.ndarray):
    """Write a rigid transform into a ROS Pose field."""

    x, y, z, w = quaternion_xyzw(transform[:3, :3])
    pose.position.x, pose.position.y, pose.position.z = [float(value) for value in transform[:3, 3]]
    pose.orientation.x = x
    pose.orientation.y = y
    pose.orientation.z = z
    pose.orientation.w = w


def skew(vector: np.ndarray) -> np.ndarray:
    x, y, z = [float(value) for value in vector]
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]], dtype=np.float64)


def twist_adjoint(base_from_source: np.ndarray) -> np.ndarray:
    """Map a source-origin [linear, angular] twist into the base origin."""

    rotation = np.asarray(base_from_source[:3, :3], dtype=np.float64)
    translation = np.asarray(base_from_source[:3, 3], dtype=np.float64)
    result = np.zeros((6, 6), dtype=np.float64)
    result[:3, :3] = rotation
    result[:3, 3:] = skew(translation) @ rotation
    result[3:, 3:] = rotation
    return result


class ZedOdometryAdapterNode(Node):
    """Republish ZED's local VIO pose and twist with base_link semantics."""

    def __init__(self):
        super().__init__("zed_odometry_adapter")
        self.declare_parameter("input_topic", "/zedx/zed_node/odom")
        self.declare_parameter("output_topic", "/localization/zed_odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("minimum_linear_speed_mps", 0.002)
        self.last_warning = ""

        self.base_frame = str(self.get_parameter("base_frame").value)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)
        self.publisher = self.create_publisher(
            Odometry, str(self.get_parameter("output_topic").value), 20
        )
        self.create_subscription(
            Odometry, str(self.get_parameter("input_topic").value), self.on_odometry, 20
        )

    def on_odometry(self, message: Odometry):
        source_frame = str(message.child_frame_id)
        if not source_frame:
            self.warn_once("ZED odometry has no child frame; cannot rotate its twist into base_link")
            return
        try:
            transform = self.tf_buffer.lookup_transform(self.base_frame, source_frame, Time())
            base_from_source = np.eye(4, dtype=np.float64)
            base_from_source[:3, :3] = rotation_from_quaternion(transform.transform.rotation)
            base_from_source[:3, 3] = [
                float(transform.transform.translation.x),
                float(transform.transform.translation.y),
                float(transform.transform.translation.z),
            ]
            odom_from_source = transform_from_pose(message.pose.pose)
        except (TransformException, ValueError) as exc:
            self.warn_once(f"waiting for valid VIO pose and TF {self.base_frame} <- {source_frame}: {exc}")
            return

        output = Odometry()
        output.header = message.header
        output.child_frame_id = self.base_frame
        odom_from_base = odom_from_source @ invert_transform(base_from_source)
        set_pose_from_transform(output.pose.pose, odom_from_base)
        source_twist = np.array(
            [
                message.twist.twist.linear.x,
                message.twist.twist.linear.y,
                message.twist.twist.linear.z,
                message.twist.twist.angular.x,
                message.twist.twist.angular.y,
                message.twist.twist.angular.z,
            ],
            dtype=np.float64,
        )
        base_twist = twist_adjoint(base_from_source) @ source_twist
        linear = base_twist[:3]
        angular = base_twist[3:]
        minimum_speed = max(0.0, float(self.get_parameter("minimum_linear_speed_mps").value))
        if float(np.linalg.norm(linear)) < minimum_speed:
            linear[:] = 0.0
        output.twist.twist.linear.x, output.twist.twist.linear.y, output.twist.twist.linear.z = linear
        output.twist.twist.angular.x, output.twist.twist.angular.y, output.twist.twist.angular.z = angular
        output.twist.covariance = self.transform_covariance(message.twist.covariance, base_from_source)
        output.pose.covariance = self.transform_covariance(message.pose.covariance, base_from_source)
        self.publisher.publish(output)
        self.last_warning = ""

    @staticmethod
    def transform_covariance(covariance, base_from_source: np.ndarray):
        source = np.asarray(covariance, dtype=np.float64).reshape(6, 6)
        transform = twist_adjoint(base_from_source)
        return (transform @ source @ transform.T).reshape(-1).tolist()

    def warn_once(self, message: str):
        if message != self.last_warning:
            self.get_logger().warn(message)
            self.last_warning = message


def main(args=None):
    rclpy.init(args=args)
    node = ZedOdometryAdapterNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
