"""Align ZED's local visual-inertial odometry to the AprilTag map frame."""

from __future__ import annotations

from collections import deque
from copy import deepcopy
import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_srvs.srv import Trigger

from .localization_math import (
    quaternion_xyzw,
    rotation_from_quaternion_xyzw,
)
from .tag_vio_alignment_math import (
    AlignmentCandidateWindow,
    blend_transform,
    interpolate_transform,
    map_from_base_from_vio,
    map_from_odom_from_tag,
)


def stamp_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def transform_from_pose(pose) -> np.ndarray:
    quaternion = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    result = np.eye(4, dtype=np.float64)
    result[:3, :3] = rotation_from_quaternion_xyzw(quaternion)
    result[:3, 3] = [float(pose.position.x), float(pose.position.y), float(pose.position.z)]
    return result


def set_pose_from_transform(pose, transform: np.ndarray):
    x, y, z, w = quaternion_xyzw(transform[:3, :3])
    pose.position.x, pose.position.y, pose.position.z = [float(value) for value in transform[:3, 3]]
    pose.orientation.x = x
    pose.orientation.y = y
    pose.orientation.z = z
    pose.orientation.w = w


class TagVioAlignmentNode(Node):
    """Use valid AprilTags to calibrate map->odom while ZED VIO carries motion."""

    def __init__(self):
        super().__init__("tag_vio_alignment")
        self.declare_parameter("tag_pose_topic", "/localization/apriltag_pose")
        self.declare_parameter("vio_odometry_topic", "/localization/zed_odom")
        self.declare_parameter("output_odometry_topic", "/localization/fused_odom")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("vio_history_duration_s", 3.0)
        self.declare_parameter("tag_vio_sync_tolerance_s", 0.12)
        # A calibration must be stable across a few camera frames before it
        # can change map->odom. This is deliberately separate from Tag
        # detection: VIO keeps publishing even while a candidate is checked.
        self.declare_parameter("alignment_confirm_frames", 4)
        self.declare_parameter("alignment_candidate_max_spread_m", 0.20)
        self.declare_parameter("alignment_candidate_max_angle_deg", 12.0)
        self.declare_parameter("alignment_candidate_max_gap_s", 0.50)
        self.declare_parameter("alignment_recalibration_threshold_m", 0.12)
        # Larger genuine relocations are requested explicitly from the web
        # Relocalize action, which resets the current alignment first.  Small
        # accepted corrections are deliberately applied gradually so control
        # consumers do not see a discontinuous global pose.
        self.declare_parameter("alignment_correction_alpha", 0.25)
        self.declare_parameter("alignment_max_correction_m", 0.75)
        self.declare_parameter("relocalize_service", "/localization/tag_vio/relocalize")

        self.map_frame = str(self.get_parameter("map_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.vio_history: deque[tuple[int, np.ndarray, Odometry]] = deque()
        self.map_from_odom: np.ndarray | None = None
        self.alignment_covariance: list[float] | None = None
        self.alignment_candidate = AlignmentCandidateWindow()
        # Global pose establishment is operator initiated.  The rest of the
        # graph, including ZED VIO, is ready immediately after startup.
        self.relocalize_requested = False
        self.relocalization_pending = False
        self.last_warning = ""

        self.output_pub = self.create_publisher(
            Odometry, str(self.get_parameter("output_odometry_topic").value), 20
        )
        self.create_subscription(
            Odometry, str(self.get_parameter("vio_odometry_topic").value), self.on_vio_odometry, 20
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            str(self.get_parameter("tag_pose_topic").value),
            self.on_tag_pose,
            20,
        )
        self.create_service(
            Trigger, str(self.get_parameter("relocalize_service").value), self.on_relocalize
        )
        self.get_logger().info(
            "Tag/VIO alignment is idle; use AprilTag map Relocalize to establish map->odom"
        )

    def on_relocalize(self, _request: Trigger.Request, response: Trigger.Response):
        """Allow the next quality-gated Tag pose to establish a fresh alignment."""

        self.map_from_odom = None
        self.alignment_covariance = None
        self.clear_alignment_candidate()
        self.relocalize_requested = True
        self.relocalization_pending = True
        self.last_warning = ""
        response.success = True
        response.message = (
            "Tag/VIO alignment reset; waiting for four trusted AprilTag observations "
            "to establish map->odom without an old-pose jump limit"
        )
        self.get_logger().info(response.message)
        return response

    def on_vio_odometry(self, message: Odometry):
        if not message.header.frame_id or message.child_frame_id != self.base_frame:
            self.warn_once(
                f"VIO odometry must be parent->{self.base_frame} with a non-empty parent frame"
            )
            return
        try:
            odom_from_base = transform_from_pose(message.pose.pose)
        except (TypeError, ValueError) as exc:
            self.warn_once(f"ZED VIO pose is invalid: {exc}")
            return
        sample_ns = stamp_ns(message.header.stamp)
        if sample_ns <= 0:
            self.warn_once("ZED VIO odometry has no valid timestamp")
            return
        if self.vio_history and sample_ns < self.vio_history[-1][0]:
            self.warn_once("ZED VIO odometry timestamp moved backwards")
            return
        if self.vio_history and sample_ns == self.vio_history[-1][0]:
            # ZED can emit an updated pose with the same image timestamp.
            # Keep the newest pose; it is not a new interpolation point.
            self.vio_history[-1] = (sample_ns, odom_from_base, deepcopy(message))
        else:
            self.vio_history.append((sample_ns, odom_from_base, deepcopy(message)))
        history_ns = int(
            max(0.1, float(self.get_parameter("vio_history_duration_s").value)) * 1e9
        )
        while self.vio_history and sample_ns - self.vio_history[0][0] > history_ns:
            self.vio_history.popleft()
        if self.map_from_odom is not None:
            self.publish_aligned_odometry(message, odom_from_base)
        self.last_warning = ""

    def on_tag_pose(self, message: PoseWithCovarianceStamped):
        if not self.relocalize_requested:
            return
        if message.header.frame_id and message.header.frame_id != self.map_frame:
            self.warn_once(
                f"AprilTag pose frame {message.header.frame_id!r} is not {self.map_frame!r}"
            )
            return
        try:
            map_from_base = transform_from_pose(message.pose.pose)
        except (TypeError, ValueError) as exc:
            self.warn_once(f"AprilTag pose is invalid: {exc}")
            return
        tag_stamp_ns = stamp_ns(message.header.stamp)
        vio = self.vio_pose_at(tag_stamp_ns)
        if vio is None:
            self.warn_once("no time-aligned ZED VIO pose for valid AprilTag observation")
            return
        _, odom_from_base, _ = vio
        observed_alignment = map_from_odom_from_tag(map_from_base, odom_from_base)
        if not self.confirm_alignment_candidate(observed_alignment, tag_stamp_ns):
            return
        confirmed_alignment = self.alignment_candidate.representative()
        self.clear_alignment_candidate()
        if self.relocalization_pending:
            # The operator has declared the previous localization inaccurate.
            # Establish the new absolute alignment regardless of displacement
            # or rotation from it; only consistency among the new observations
            # is required.
            self.map_from_odom = confirmed_alignment
            self.alignment_covariance = list(message.pose.covariance)
            self.relocalization_pending = False
            self.get_logger().info(
                "AprilTag established a relocalized map->odom alignment "
                "without applying the old-pose jump limit"
            )
        elif self.map_from_odom is None:
            self.map_from_odom = confirmed_alignment
            self.alignment_covariance = list(message.pose.covariance)
            self.get_logger().info("AprilTag established a confirmed initial map->odom alignment")
        else:
            correction_m = float(
                np.linalg.norm(confirmed_alignment[:3, 3] - self.map_from_odom[:3, 3])
            )
            maximum_correction_m = max(
                0.0, float(self.get_parameter("alignment_max_correction_m").value)
            )
            if maximum_correction_m > 0.0 and correction_m > maximum_correction_m:
                self.warn_once(
                    f"AprilTag alignment correction {correction_m:.2f}m exceeds "
                    f"{maximum_correction_m:.2f}m limit"
                )
                return
            minimum_correction_m = max(
                0.0, float(self.get_parameter("alignment_recalibration_threshold_m").value)
            )
            if correction_m < minimum_correction_m:
                return
            alpha = min(
                1.0,
                max(0.0, float(self.get_parameter("alignment_correction_alpha").value)),
            )
            self.map_from_odom = blend_transform(self.map_from_odom, confirmed_alignment, alpha)
            self.alignment_covariance = list(message.pose.covariance)
            self.get_logger().info(
                f"AprilTag recalibrated map->odom by {correction_m:.2f}m"
            )
        latest_stamp_ns, latest_odom_from_base, latest_message = self.vio_history[-1]
        del latest_stamp_ns
        self.publish_aligned_odometry(latest_message, latest_odom_from_base)
        self.last_warning = ""

    def confirm_alignment_candidate(self, observed: np.ndarray, stamp_ns: int) -> bool:
        """Require a short, spatially and angularly consistent Tag cluster."""

        maximum_gap_ns = int(
            max(0.0, float(self.get_parameter("alignment_candidate_max_gap_s").value)) * 1e9
        )
        maximum_spread_m = max(
            0.0, float(self.get_parameter("alignment_candidate_max_spread_m").value)
        )
        maximum_angle_rad = math.radians(
            max(0.0, float(self.get_parameter("alignment_candidate_max_angle_deg").value))
        )
        return self.alignment_candidate.add(
            observed,
            stamp_ns,
            confirm_frames=int(self.get_parameter("alignment_confirm_frames").value),
            max_spread_m=maximum_spread_m,
            max_angle_rad=maximum_angle_rad,
            max_gap_ns=maximum_gap_ns,
        )

    def clear_alignment_candidate(self):
        self.alignment_candidate.clear()

    def vio_pose_at(self, target_ns: int) -> tuple[int, np.ndarray, Odometry] | None:
        if not self.vio_history:
            return None
        tolerance_ns = int(
            max(0.0, float(self.get_parameter("tag_vio_sync_tolerance_s").value)) * 1e9
        )
        before = None
        after = None
        for sample in self.vio_history:
            if sample[0] <= target_ns:
                before = sample
            if sample[0] >= target_ns:
                after = sample
                break
        if before is None or after is None:
            nearest = self.vio_history[0] if before is None else self.vio_history[-1]
            return nearest if abs(nearest[0] - target_ns) <= tolerance_ns else None
        if before[0] == after[0]:
            return before
        if max(target_ns - before[0], after[0] - target_ns) > tolerance_ns:
            return None
        fraction = (target_ns - before[0]) / (after[0] - before[0])
        interpolated = interpolate_transform(before[1], after[1], fraction)
        return target_ns, interpolated, before[2]

    def publish_aligned_odometry(self, vio_message: Odometry, odom_from_base: np.ndarray):
        if self.map_from_odom is None:
            return
        output = Odometry()
        output.header = deepcopy(vio_message.header)
        output.header.frame_id = self.map_frame
        output.child_frame_id = self.base_frame
        set_pose_from_transform(
            output.pose.pose, map_from_base_from_vio(self.map_from_odom, odom_from_base)
        )
        output.pose.covariance = self.combined_pose_covariance(vio_message.pose.covariance)
        output.twist = deepcopy(vio_message.twist)
        self.output_pub.publish(output)

    def combined_pose_covariance(self, vio_covariance) -> list[float]:
        output = list(vio_covariance)
        if self.alignment_covariance is None or len(output) != 36:
            return output
        for index in (0, 7, 14, 21, 28, 35):
            local = float(output[index]) if math.isfinite(float(output[index])) else 0.0
            tag = float(self.alignment_covariance[index])
            output[index] = max(0.0, local) + (max(0.0, tag) if math.isfinite(tag) else 0.0)
        return output

    def warn_once(self, message: str):
        if message != self.last_warning:
            self.get_logger().warn(message)
            self.last_warning = message


def main(args=None):
    rclpy.init(args=args)
    node = TagVioAlignmentNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
