"""Align ZED's local visual-inertial odometry to the AprilTag map frame."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger

from .localization_math import (
    quaternion_xyzw,
    rotation_from_quaternion_xyzw,
)
from .tag_vio_alignment_math import (
    AlignmentCandidateWindow,
    aligned_pose_covariance,
    blend_transform,
    interpolate_transform,
    map_from_base_from_vio,
    map_from_odom_from_tag,
    rotation_distance_rad,
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


@dataclass(frozen=True)
class VioMessageSnapshot:
    """Only the Odometry fields needed after timestamp interpolation."""

    stamp_sec: int
    stamp_nanosec: int
    pose_covariance: tuple[float, ...]
    linear_velocity: tuple[float, float, float]
    angular_velocity: tuple[float, float, float]
    twist_covariance: tuple[float, ...]


def snapshot_vio_message(message: Odometry) -> VioMessageSnapshot:
    twist = message.twist.twist
    return VioMessageSnapshot(
        stamp_sec=int(message.header.stamp.sec),
        stamp_nanosec=int(message.header.stamp.nanosec),
        pose_covariance=tuple(float(value) for value in message.pose.covariance),
        linear_velocity=(
            float(twist.linear.x),
            float(twist.linear.y),
            float(twist.linear.z),
        ),
        angular_velocity=(
            float(twist.angular.x),
            float(twist.angular.y),
            float(twist.angular.z),
        ),
        twist_covariance=tuple(float(value) for value in message.twist.covariance),
    )


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
        self.declare_parameter("alignment_recalibration_threshold_m", 0.05)
        self.declare_parameter("alignment_recalibration_threshold_deg", 2.0)
        # Larger genuine relocations are requested explicitly from the web
        # Relocalize action, which resets the current alignment first.  Small
        # accepted corrections are deliberately applied gradually so control
        # consumers do not see a discontinuous global pose.
        self.declare_parameter("alignment_correction_alpha", 0.25)
        self.declare_parameter("alignment_max_correction_m", 0.75)
        self.declare_parameter("alignment_max_correction_angle_deg", 20.0)
        self.declare_parameter("relocalize_service", "/localization/tag_vio/relocalize")

        self.map_frame = str(self.get_parameter("map_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.vio_history: deque[tuple[int, np.ndarray, VioMessageSnapshot]] = deque()
        self.map_from_odom: np.ndarray | None = None
        self.alignment_covariance: list[float] | None = None
        self.alignment_candidate = AlignmentCandidateWindow()
        # Global pose establishment is operator initiated.  The rest of the
        # graph, including ZED VIO, is ready immediately after startup.
        self.relocalize_requested = False
        self.relocalization_pending = False
        self.last_warning = ""

        self.latest_visual_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.output_pub = self.create_publisher(
            Odometry,
            str(self.get_parameter("output_odometry_topic").value),
            self.latest_visual_qos,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("vio_odometry_topic").value),
            self.on_vio_odometry,
            self.latest_visual_qos,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            str(self.get_parameter("tag_pose_topic").value),
            self.on_tag_pose,
            self.latest_visual_qos,
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
            # Camera restart/time reset also invalidates the old local odom
            # origin. Fail closed and permit a new operator relocalization
            # instead of rejecting every future sample forever.
            self.vio_history.clear()
            self.map_from_odom = None
            self.alignment_covariance = None
            self.clear_alignment_candidate()
            self.relocalize_requested = False
            self.relocalization_pending = False
            self.warn_once(
                "ZED VIO timestamp moved backwards; alignment cleared and "
                "operator relocalization is required"
            )
        snapshot = snapshot_vio_message(message)
        if self.vio_history and sample_ns == self.vio_history[-1][0]:
            # ZED can emit an updated pose with the same image timestamp.
            # Keep the newest pose; it is not a new interpolation point.
            self.vio_history[-1] = (sample_ns, odom_from_base, snapshot)
        else:
            self.vio_history.append((sample_ns, odom_from_base, snapshot))
        history_ns = int(
            max(0.1, float(self.get_parameter("vio_history_duration_s").value)) * 1e9
        )
        while self.vio_history and sample_ns - self.vio_history[0][0] > history_ns:
            self.vio_history.popleft()
        if self.map_from_odom is not None:
            self.publish_aligned_odometry(snapshot, odom_from_base)
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
        _, odom_from_base, vio_snapshot = vio
        observed_alignment = map_from_odom_from_tag(map_from_base, odom_from_base)
        if not self.confirm_alignment_candidate(observed_alignment, tag_stamp_ns):
            return
        confirmed_alignment = self.alignment_candidate.representative()
        tag_and_anchor_covariance = aligned_pose_covariance(
            message.pose.covariance,
            vio_snapshot.pose_covariance,
            confirmed_alignment,
        )
        confirmed_covariance = aligned_pose_covariance(
            tag_and_anchor_covariance,
            self.alignment_candidate.covariance(),
            np.eye(4, dtype=np.float64),
        )
        self.clear_alignment_candidate()
        if self.relocalization_pending:
            # The operator has declared the previous localization inaccurate.
            # Establish the new absolute alignment regardless of displacement
            # or rotation from it; only consistency among the new observations
            # is required.
            self.map_from_odom = confirmed_alignment
            self.alignment_covariance = confirmed_covariance
            self.relocalization_pending = False
            self.get_logger().info(
                "AprilTag established a relocalized map->odom alignment "
                "without applying the old-pose jump limit"
            )
        elif self.map_from_odom is None:
            self.map_from_odom = confirmed_alignment
            self.alignment_covariance = confirmed_covariance
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
            correction_angle_deg = math.degrees(
                rotation_distance_rad(self.map_from_odom, confirmed_alignment)
            )
            maximum_correction_angle_deg = max(
                0.0,
                float(
                    self.get_parameter("alignment_max_correction_angle_deg").value
                ),
            )
            if (
                maximum_correction_angle_deg > 0.0
                and correction_angle_deg > maximum_correction_angle_deg
            ):
                self.warn_once(
                    "AprilTag alignment rotation correction "
                    f"{correction_angle_deg:.1f}deg exceeds "
                    f"{maximum_correction_angle_deg:.1f}deg limit"
                )
                return
            minimum_correction_m = max(
                0.0, float(self.get_parameter("alignment_recalibration_threshold_m").value)
            )
            minimum_correction_angle_deg = max(
                0.0,
                float(
                    self.get_parameter(
                        "alignment_recalibration_threshold_deg"
                    ).value
                ),
            )
            if (
                correction_m < minimum_correction_m
                and correction_angle_deg < minimum_correction_angle_deg
            ):
                return
            alpha = min(
                1.0,
                max(0.0, float(self.get_parameter("alignment_correction_alpha").value)),
            )
            self.map_from_odom = blend_transform(self.map_from_odom, confirmed_alignment, alpha)
            if self.alignment_covariance is None:
                self.alignment_covariance = confirmed_covariance
            else:
                previous_covariance = np.asarray(
                    self.alignment_covariance, dtype=np.float64
                )
                observed_covariance = np.asarray(
                    confirmed_covariance, dtype=np.float64
                )
                self.alignment_covariance = (
                    (1.0 - alpha) * previous_covariance
                    + alpha * observed_covariance
                ).tolist()
            # The correction is reflected in the aligned odometry and status
            # topics; avoid per-correction console I/O on the fusion hot path.
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

    def vio_pose_at(
        self, target_ns: int
    ) -> tuple[int, np.ndarray, VioMessageSnapshot] | None:
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
        # Do not turn a delayed Tag into an alignment bias by extrapolating a
        # nearest VIO pose. Detection latency is harmless because history keeps
        # the timestamp-bracketing samples.
        if before is None or after is None:
            return None
        if before[0] == after[0]:
            return before
        if max(target_ns - before[0], after[0] - target_ns) > tolerance_ns:
            return None
        fraction = (target_ns - before[0]) / (after[0] - before[0])
        interpolated = interpolate_transform(before[1], after[1], fraction)
        return target_ns, interpolated, before[2]

    def publish_aligned_odometry(
        self, vio_message: VioMessageSnapshot, odom_from_base: np.ndarray
    ):
        if self.map_from_odom is None:
            return
        output = Odometry()
        output.header.stamp.sec = vio_message.stamp_sec
        output.header.stamp.nanosec = vio_message.stamp_nanosec
        output.header.frame_id = self.map_frame
        output.child_frame_id = self.base_frame
        set_pose_from_transform(
            output.pose.pose, map_from_base_from_vio(self.map_from_odom, odom_from_base)
        )
        output.pose.covariance = self.combined_pose_covariance(
            vio_message.pose_covariance
        )
        linear = vio_message.linear_velocity
        angular = vio_message.angular_velocity
        output.twist.twist.linear.x = linear[0]
        output.twist.twist.linear.y = linear[1]
        output.twist.twist.linear.z = linear[2]
        output.twist.twist.angular.x = angular[0]
        output.twist.twist.angular.y = angular[1]
        output.twist.twist.angular.z = angular[2]
        output.twist.covariance = list(vio_message.twist_covariance)
        self.output_pub.publish(output)

    def combined_pose_covariance(self, vio_covariance) -> list[float]:
        output = list(vio_covariance)
        if self.alignment_covariance is None or len(output) != 36:
            return output
        return aligned_pose_covariance(
            self.alignment_covariance,
            output,
            self.map_from_odom,
        )

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
