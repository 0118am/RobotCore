"""Publish the measured fixed sensor mounting transforms for the vehicle."""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from .localization_math import quaternion_xyzw, rpy_matrix


class VehicleFramesNode(Node):
    """Publish the fixed base_link -> ZED camera frames used by localisation."""

    def __init__(self):
        super().__init__("vehicle_frames")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("camera_link_frame", "zedx_camera_link")
        self.declare_parameter("camera_optical_frame", "front_camera_optical_frame")
        # Values are measured from the centre of mass (base_link) in ROS FLU.
        self.declare_parameter("base_to_camera_translation_m", [0.236, 0.027, 0.016])
        # camera_optical: +X right, +Y down, +Z forward.
        self.declare_parameter(
            "base_to_camera_optical_rpy_rad", [-math.pi / 2.0, 0.0, -math.pi / 2.0]
        )

        self.broadcaster = StaticTransformBroadcaster(self)
        self.broadcaster.sendTransform(
            [
                self.make_transform(
                    str(self.get_parameter("base_frame").value),
                    str(self.get_parameter("camera_link_frame").value),
                    self.get_parameter("base_to_camera_translation_m").value,
                    [0.0, 0.0, 0.0],
                ),
                self.make_transform(
                    str(self.get_parameter("base_frame").value),
                    str(self.get_parameter("camera_optical_frame").value),
                    self.get_parameter("base_to_camera_translation_m").value,
                    self.get_parameter("base_to_camera_optical_rpy_rad").value,
                ),
            ]
        )

    def make_transform(self, parent: str, child: str, translation, rpy) -> TransformStamped:
        if len(translation) != 3 or len(rpy) != 3:
            raise ValueError(f"TF {parent}->{child} requires three translation and RPY values")
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = parent
        transform.child_frame_id = child
        transform.transform.translation.x = float(translation[0])
        transform.transform.translation.y = float(translation[1])
        transform.transform.translation.z = float(translation[2])
        x, y, z, w = quaternion_xyzw(rpy_matrix(*[float(value) for value in rpy]))
        transform.transform.rotation.x = x
        transform.transform.rotation.y = y
        transform.transform.rotation.z = z
        transform.transform.rotation.w = w
        return transform


def main(args=None):
    rclpy.init(args=args)
    node = VehicleFramesNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
