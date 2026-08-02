"""Publish the Tag-aligned VIO/EKF estimate in the body-state contract.

The node does not perform a second Kalman filter. It adapts the canonical
``/localization/fused_odom`` estimate, validates its provenance with
time-aligned source observations, and publishes quantitative localisation
health alongside ``BodyState``.
"""

from __future__ import annotations

from collections import deque
from copy import deepcopy
from dataclasses import dataclass
import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, Int32

from eup_interfaces.msg import AprilTagPoseStatus, BodyState, LocalizationStatus

from .localization_math import rotation_from_quaternion_xyzw
from .tag_vio_alignment_math import interpolate_transform, rotation_distance_rad


def finite(value) -> bool:
    return math.isfinite(float(value))


def stamp_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def quaternion_valid(orientation) -> bool:
    values = (
        float(orientation.w),
        float(orientation.x),
        float(orientation.y),
        float(orientation.z),
    )
    return all(math.isfinite(value) for value in values) and any(
        abs(value) > 1e-9 for value in values
    )


def transform_from_pose(pose) -> np.ndarray:
    result = np.eye(4, dtype=np.float64)
    result[:3, :3] = rotation_from_quaternion_xyzw(
        [
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ]
    )
    result[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
    if not np.isfinite(result).all():
        raise ValueError("pose contains non-finite values")
    return result


@dataclass
class TagObservation:
    message: PoseWithCovarianceStamped
    arrival_ns: int
    transport_delay_s: float
    observation_class: str


class SensorFusionNode(Node):
    """Adapt map-frame Tag-calibrated ZED VIO output to :class:`BodyState`."""

    def __init__(self):
        super().__init__("sensor_fusion_node")
        self.declare_parameter("body_state_topic", "/robot/body_state")
        self.declare_parameter("localization_status_topic", "/localization/status")
        self.declare_parameter("depth_input_topic", "")
        self.declare_parameter("altitude_input_topic", "")
        self.declare_parameter("depth_max_age_s", 0.50)
        self.declare_parameter("altitude_max_age_s", 0.50)
        self.declare_parameter("body_frame_id", "map")
        self.declare_parameter("localization_pose_topic", "/localization/apriltag_pose")
        self.declare_parameter(
            "degraded_localization_pose_topic",
            "/localization/apriltag_pose_degraded",
        )
        self.declare_parameter("detected_tag_count_topic", "/localization/apriltag/detected_count")
        self.declare_parameter(
            "apriltag_pose_status_topic", "/localization/apriltag/pose_status"
        )
        self.declare_parameter("apriltag_pose_status_max_age_s", 0.50)
        self.declare_parameter("localization_max_age_s", 0.20)
        self.declare_parameter("fused_odometry_topic", "/localization/fused_odom")
        self.declare_parameter("fused_history_duration_s", 2.0)
        self.declare_parameter("tag_fused_sync_tolerance_s", 0.08)
        self.declare_parameter("zed_odometry_topic", "/localization/zed_odom")
        self.declare_parameter("zed_odometry_max_age_s", 0.35)
        self.declare_parameter("external_imu_topic", "/sensors/external_imu")
        self.declare_parameter("external_imu_max_age_s", 0.20)
        self.declare_parameter("tag_vio_disagreement_m", 0.50)
        self.declare_parameter("tag_vio_disagreement_deg", 10.0)
        self.declare_parameter("rate_window_s", 5.0)

        self.depth_m = math.nan
        self.depth_arrival_ns = 0
        self.altitude_m = math.nan
        self.altitude_arrival_ns = 0
        self.primary_localization: TagObservation | None = None
        self.degraded_localization: TagObservation | None = None
        self.zed_odometry_arrival_ns = 0
        self.zed_odometry_stamp_ns = 0
        self.zed_transport_delay_s = math.nan
        self.external_imu_arrival_ns = 0
        self.external_imu_stamp_ns = 0
        self.detected_tag_count = 0
        self.apriltag_pose_status: AprilTagPoseStatus | None = None
        self.apriltag_pose_status_arrival_ns = 0
        self.last_tag_stamp_ns = 0
        self.last_tag_transport_delay_s = math.nan
        self.last_absolute_fix_stamp_ns = 0
        self.last_status_warning = ""
        self.fused_history: deque[tuple[int, np.ndarray]] = deque()
        self.vio_arrivals: deque[int] = deque()
        self.tag_arrivals: deque[int] = deque()
        self.fused_arrivals: deque[int] = deque()
        self.apriltag_frame_arrivals: deque[int] = deque()

        self.body_state_topic = str(self.get_parameter("body_state_topic").value)
        self.body_frame_id = str(self.get_parameter("body_frame_id").value)
        self.fused_odometry_topic = str(self.get_parameter("fused_odometry_topic").value)
        self.zed_odometry_topic = str(self.get_parameter("zed_odometry_topic").value)
        if not self.fused_odometry_topic:
            raise ValueError("fused_odometry_topic must be set")

        # Visual measurements are latest-value streams. BEST_EFFORT is required
        # to match the ZED adapter publisher and depth one prevents stale queues.
        self.latest_sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.fused_state_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.body_pub = self.create_publisher(BodyState, self.body_state_topic, 10)
        self.status_pub = self.create_publisher(
            LocalizationStatus,
            str(self.get_parameter("localization_status_topic").value),
            10,
        )

        localization_topic = str(self.get_parameter("localization_pose_topic").value)
        if localization_topic:
            self.create_subscription(
                PoseWithCovarianceStamped,
                localization_topic,
                self.on_primary_localization_pose,
                self.latest_sensor_qos,
            )
        degraded_topic = str(
            self.get_parameter("degraded_localization_pose_topic").value
        )
        if degraded_topic:
            self.create_subscription(
                PoseWithCovarianceStamped,
                degraded_topic,
                self.on_degraded_localization_pose,
                self.latest_sensor_qos,
            )
        count_topic = str(self.get_parameter("detected_tag_count_topic").value)
        if count_topic:
            self.create_subscription(
                Int32, count_topic, self.on_detected_tag_count, self.latest_sensor_qos
            )
        apriltag_status_topic = str(
            self.get_parameter("apriltag_pose_status_topic").value
        )
        if apriltag_status_topic:
            self.create_subscription(
                AprilTagPoseStatus,
                apriltag_status_topic,
                self.on_apriltag_pose_status,
                self.latest_sensor_qos,
            )
        self.create_subscription(
            Odometry,
            self.fused_odometry_topic,
            self.on_fused_odometry,
            self.fused_state_qos,
        )
        if self.zed_odometry_topic:
            self.create_subscription(
                Odometry,
                self.zed_odometry_topic,
                self.on_zed_odometry,
                self.latest_sensor_qos,
            )
        external_imu_topic = str(self.get_parameter("external_imu_topic").value)
        if external_imu_topic:
            self.create_subscription(
                Imu,
                external_imu_topic,
                self.on_external_imu,
                self.latest_sensor_qos,
            )

        depth_topic = str(self.get_parameter("depth_input_topic").value)
        if depth_topic:
            self.create_subscription(
                Float32, depth_topic, self.on_depth, self.latest_sensor_qos
            )
        altitude_topic = str(self.get_parameter("altitude_input_topic").value)
        if altitude_topic:
            self.create_subscription(
                Float32, altitude_topic, self.on_altitude, self.latest_sensor_qos
            )

        self.get_logger().info(
            "Sensor fusion publishing body state and quantitative localisation "
            f"status from {self.fused_odometry_topic}"
        )

    def on_depth(self, msg: Float32):
        value = float(msg.data)
        if finite(value):
            self.depth_m = value
            self.depth_arrival_ns = self.get_clock().now().nanoseconds

    def on_altitude(self, msg: Float32):
        value = float(msg.data)
        if finite(value):
            self.altitude_m = value
            self.altitude_arrival_ns = self.get_clock().now().nanoseconds

    def on_detected_tag_count(self, msg: Int32):
        self.detected_tag_count = max(0, int(msg.data))

    def on_apriltag_pose_status(self, msg: AprilTagPoseStatus):
        source_stamp_ns = stamp_ns(msg.header.stamp)
        if source_stamp_ns <= 0:
            return
        if (
            self.apriltag_pose_status is not None
            and source_stamp_ns <= stamp_ns(self.apriltag_pose_status.header.stamp)
        ):
            return
        self.apriltag_pose_status = deepcopy(msg)
        self.apriltag_pose_status_arrival_ns = self.get_clock().now().nanoseconds
        self.observe_rate(
            self.apriltag_frame_arrivals, self.apriltag_pose_status_arrival_ns
        )
        self.detected_tag_count = max(0, int(msg.detected_tag_count))

    def on_primary_localization_pose(self, msg: PoseWithCovarianceStamped):
        self.record_localization_pose(msg, "primary")

    def on_degraded_localization_pose(self, msg: PoseWithCovarianceStamped):
        self.record_localization_pose(msg, "degraded")

    def record_localization_pose(
        self, msg: PoseWithCovarianceStamped, observation_class: str
    ):
        if not quaternion_valid(msg.pose.pose.orientation):
            return
        source_stamp_ns = stamp_ns(msg.header.stamp)
        if source_stamp_ns <= 0:
            return
        previous = (
            self.primary_localization
            if observation_class == "primary"
            else self.degraded_localization
        )
        if previous is not None and source_stamp_ns <= stamp_ns(previous.message.header.stamp):
            return
        now_ns = self.get_clock().now().nanoseconds
        observation = TagObservation(
            message=deepcopy(msg),
            arrival_ns=now_ns,
            transport_delay_s=max(0.0, (now_ns - source_stamp_ns) * 1e-9),
            observation_class=observation_class,
        )
        if observation_class == "primary":
            self.primary_localization = observation
        else:
            self.degraded_localization = observation
        if source_stamp_ns >= self.last_tag_stamp_ns:
            self.last_tag_stamp_ns = source_stamp_ns
            self.last_tag_transport_delay_s = observation.transport_delay_s
        self.observe_rate(self.tag_arrivals, now_ns)

    def on_zed_odometry(self, msg: Odometry):
        """Record a recent, source-stamped finite VIO velocity."""

        values = (
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z,
        )
        source_stamp_ns = stamp_ns(msg.header.stamp)
        if not all(finite(value) for value in values) or source_stamp_ns <= 0:
            return
        # Repeated or queued old samples must not refresh VIO validity.
        if source_stamp_ns <= self.zed_odometry_stamp_ns:
            return
        now_ns = self.get_clock().now().nanoseconds
        self.zed_odometry_stamp_ns = source_stamp_ns
        self.zed_odometry_arrival_ns = now_ns
        self.zed_transport_delay_s = max(0.0, (now_ns - source_stamp_ns) * 1e-9)
        self.observe_rate(self.vio_arrivals, now_ns)

    def on_external_imu(self, msg: Imu):
        values = (
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z,
        )
        source_stamp_ns = stamp_ns(msg.header.stamp)
        if not all(finite(value) for value in values) or source_stamp_ns <= 0:
            return
        if source_stamp_ns <= self.external_imu_stamp_ns:
            return
        self.external_imu_stamp_ns = source_stamp_ns
        self.external_imu_arrival_ns = self.get_clock().now().nanoseconds

    def on_fused_odometry(self, odometry: Odometry):
        if not quaternion_valid(odometry.pose.pose.orientation):
            return
        source_stamp_ns = stamp_ns(odometry.header.stamp)
        if source_stamp_ns <= 0:
            return
        try:
            fused_transform = transform_from_pose(odometry.pose.pose)
        except ValueError:
            return
        now_ns = self.get_clock().now().nanoseconds
        self.record_fused_pose(source_stamp_ns, fused_transform)
        self.observe_rate(self.fused_arrivals, now_ns)

        vio_fresh = self.zed_odometry_is_fresh(now_ns)
        imu_fresh = self.external_imu_is_fresh(now_ns)
        tag = self.fresh_localization_pose(now_ns)
        consistent, translation_residual_m, angle_residual_deg, rejection_reason = (
            self.tag_matches_filter(tag)
        )
        tag_accepted = tag is not None and consistent and vio_fresh
        tag_rejected = tag is not None and not tag_accepted
        if tag is not None and consistent and not vio_fresh:
            rejection_reason = "time-aligned Tag is valid but ZED VIO is stale"
        apriltag_status = self.fresh_apriltag_pose_status(now_ns)
        if (
            tag is None
            and apriltag_status is not None
            and not apriltag_status.pose_published
        ):
            rejection_reason = str(apriltag_status.rejection_reason)

        body = self.make_body_state_from_fused_odometry(odometry, now_ns)
        body.linear_velocity_valid = body.linear_velocity_valid and vio_fresh
        body.state_valid = tag_accepted
        body.position_estimated = vio_fresh and not tag_accepted
        body.localization_source = self.localization_source(
            tag_accepted=tag_accepted,
            tag_rejected=tag_rejected,
            vio_available=vio_fresh,
            imu_available=imu_fresh,
        )
        if tag_accepted:
            self.last_absolute_fix_stamp_ns = max(
                self.last_absolute_fix_stamp_ns,
                stamp_ns(tag.message.header.stamp),
            )
        if tag_rejected and rejection_reason:
            self.warn_status(rejection_reason)
        else:
            self.last_status_warning = ""

        self.body_pub.publish(body)
        self.status_pub.publish(
            self.make_localization_status(
                odometry=odometry,
                body=body,
                now_ns=now_ns,
                tag=tag,
                tag_consistent=consistent,
                tag_accepted=tag_accepted,
                translation_residual_m=translation_residual_m,
                angle_residual_deg=angle_residual_deg,
                rejection_reason=rejection_reason,
                apriltag_status=apriltag_status,
            )
        )

    def record_fused_pose(self, source_stamp_ns: int, transform: np.ndarray):
        if self.fused_history and source_stamp_ns < self.fused_history[-1][0]:
            # A clock/filter reset invalidates interpolation across the jump.
            self.fused_history.clear()
        sample = (source_stamp_ns, transform.copy())
        if self.fused_history and source_stamp_ns == self.fused_history[-1][0]:
            self.fused_history[-1] = sample
        else:
            self.fused_history.append(sample)
        history_ns = int(
            max(0.1, float(self.get_parameter("fused_history_duration_s").value))
            * 1e9
        )
        while (
            self.fused_history
            and source_stamp_ns - self.fused_history[0][0] > history_ns
        ):
            self.fused_history.popleft()

    def fused_pose_at(self, target_ns: int) -> np.ndarray | None:
        """Interpolate the filter pose at a Tag source timestamp, without extrapolation."""

        before = None
        after = None
        for sample in self.fused_history:
            if sample[0] <= target_ns:
                before = sample
            if sample[0] >= target_ns:
                after = sample
                break
        if before is None or after is None:
            return None
        tolerance_ns = int(
            max(0.0, float(self.get_parameter("tag_fused_sync_tolerance_s").value))
            * 1e9
        )
        if max(target_ns - before[0], after[0] - target_ns) > tolerance_ns:
            return None
        if before[0] == after[0]:
            return before[1]
        fraction = (target_ns - before[0]) / (after[0] - before[0])
        return interpolate_transform(before[1], after[1], fraction)

    def make_body_state_from_fused_odometry(
        self, odometry: Odometry, now_ns: int
    ) -> BodyState:
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
        body.depth_m = self.fresh_scalar(
            self.depth_m,
            self.depth_arrival_ns,
            now_ns,
            float(self.get_parameter("depth_max_age_s").value),
        )
        body.altitude_m = self.fresh_scalar(
            self.altitude_m,
            self.altitude_arrival_ns,
            now_ns,
            float(self.get_parameter("altitude_max_age_s").value),
        )
        return body

    @staticmethod
    def fresh_scalar(value: float, arrival_ns: int, now_ns: int, max_age_s: float) -> float:
        if not finite(value) or arrival_ns <= 0:
            return math.nan
        if max_age_s > 0.0 and now_ns - arrival_ns > int(max_age_s * 1e9):
            return math.nan
        return float(value)

    def tag_matches_filter(
        self, tag: TagObservation | None
    ) -> tuple[bool, float, float, str]:
        if tag is None:
            return False, math.nan, math.nan, ""
        tag_stamp_ns = stamp_ns(tag.message.header.stamp)
        predicted = self.fused_pose_at(tag_stamp_ns)
        if predicted is None:
            return (
                False,
                math.nan,
                math.nan,
                "no time-aligned EKF pose for fresh AprilTag observation",
            )
        try:
            observed = transform_from_pose(tag.message.pose.pose)
        except ValueError:
            return False, math.nan, math.nan, "AprilTag pose is non-finite"
        translation_residual_m = float(
            np.linalg.norm(predicted[:3, 3] - observed[:3, 3])
        )
        angle_residual_deg = math.degrees(rotation_distance_rad(predicted, observed))
        maximum_translation_m = max(
            0.0, float(self.get_parameter("tag_vio_disagreement_m").value)
        )
        maximum_angle_deg = max(
            0.0, float(self.get_parameter("tag_vio_disagreement_deg").value)
        )
        translation_ok = (
            maximum_translation_m == 0.0
            or translation_residual_m <= maximum_translation_m
        )
        angle_ok = maximum_angle_deg == 0.0 or angle_residual_deg <= maximum_angle_deg
        if translation_ok and angle_ok:
            return True, translation_residual_m, angle_residual_deg, ""
        return (
            False,
            translation_residual_m,
            angle_residual_deg,
            "fresh AprilTag disagrees with time-aligned Tag/VIO estimate by "
            f"{translation_residual_m:.2f}m/{angle_residual_deg:.1f}deg",
        )

    def localization_source(
        self,
        *,
        tag_accepted: bool,
        tag_rejected: bool,
        vio_available: bool,
        imu_available: bool,
    ) -> str:
        suffix = "+External IMU" if imu_available else ""
        if tag_accepted:
            return f"Tag+ZED VIO{suffix}"
        if vio_available:
            source = f"ZED VIO{suffix}"
            return f"{source} (Tag rejected)" if tag_rejected else source
        return "localization unavailable"

    def message_is_fresh(
        self,
        *,
        arrival_ns: int,
        source_stamp_ns: int,
        now_ns: int,
        maximum_age_s: float,
    ) -> bool:
        if arrival_ns <= 0 or source_stamp_ns <= 0:
            return False
        if maximum_age_s <= 0.0:
            return True
        maximum_age_ns = int(maximum_age_s * 1e9)
        arrival_age_ns = now_ns - arrival_ns
        source_age_ns = now_ns - source_stamp_ns
        return (
            0 <= arrival_age_ns <= maximum_age_ns
            and 0 <= source_age_ns <= maximum_age_ns
        )

    def zed_odometry_is_fresh(self, now_ns: int) -> bool:
        return self.message_is_fresh(
            arrival_ns=self.zed_odometry_arrival_ns,
            source_stamp_ns=self.zed_odometry_stamp_ns,
            now_ns=now_ns,
            maximum_age_s=max(
                0.0, float(self.get_parameter("zed_odometry_max_age_s").value)
            ),
        )

    def external_imu_is_fresh(self, now_ns: int) -> bool:
        return self.message_is_fresh(
            arrival_ns=self.external_imu_arrival_ns,
            source_stamp_ns=self.external_imu_stamp_ns,
            now_ns=now_ns,
            maximum_age_s=max(
                0.0, float(self.get_parameter("external_imu_max_age_s").value)
            ),
        )

    def fresh_localization_pose(self, now_ns: int) -> TagObservation | None:
        maximum_age_s = max(
            0.0, float(self.get_parameter("localization_max_age_s").value)
        )
        candidates = []
        for observation in (self.primary_localization, self.degraded_localization):
            if observation is None:
                continue
            source_stamp_ns = stamp_ns(observation.message.header.stamp)
            if self.message_is_fresh(
                arrival_ns=observation.arrival_ns,
                source_stamp_ns=source_stamp_ns,
                now_ns=now_ns,
                maximum_age_s=maximum_age_s,
            ):
                # Prefer primary when two classes share the same image stamp.
                priority = 1 if observation.observation_class == "primary" else 0
                candidates.append((source_stamp_ns, priority, observation))
        return max(candidates, default=(0, 0, None))[2]

    def fresh_apriltag_pose_status(
        self, now_ns: int
    ) -> AprilTagPoseStatus | None:
        if self.apriltag_pose_status is None:
            return None
        if self.message_is_fresh(
            arrival_ns=self.apriltag_pose_status_arrival_ns,
            source_stamp_ns=stamp_ns(self.apriltag_pose_status.header.stamp),
            now_ns=now_ns,
            maximum_age_s=max(
                0.0,
                float(
                    self.get_parameter("apriltag_pose_status_max_age_s").value
                ),
            ),
        ):
            return self.apriltag_pose_status
        return None

    def observe_rate(self, samples: deque[int], arrival_ns: int):
        samples.append(arrival_ns)
        window_ns = int(
            max(0.5, float(self.get_parameter("rate_window_s").value)) * 1e9
        )
        while samples and arrival_ns - samples[0] > window_ns:
            samples.popleft()

    def rate_hz(self, samples: deque[int], now_ns: int) -> float:
        window_ns = int(
            max(0.5, float(self.get_parameter("rate_window_s").value)) * 1e9
        )
        while samples and now_ns - samples[0] > window_ns:
            samples.popleft()
        if len(samples) < 2 or samples[-1] <= samples[0]:
            return 0.0
        return (len(samples) - 1) * 1e9 / (samples[-1] - samples[0])

    @staticmethod
    def age_s(now_ns: int, source_stamp_ns: int) -> float:
        if source_stamp_ns <= 0:
            return math.inf
        return max(0.0, (now_ns - source_stamp_ns) * 1e-9)

    @staticmethod
    def set_time_from_ns(target, value_ns: int):
        if value_ns <= 0:
            target.sec = 0
            target.nanosec = 0
            return
        target.sec = int(value_ns // 1_000_000_000)
        target.nanosec = int(value_ns % 1_000_000_000)

    def make_localization_status(
        self,
        *,
        odometry: Odometry,
        body: BodyState,
        now_ns: int,
        tag: TagObservation | None,
        tag_consistent: bool,
        tag_accepted: bool,
        translation_residual_m: float,
        angle_residual_deg: float,
        rejection_reason: str,
        apriltag_status: AprilTagPoseStatus | None,
    ) -> LocalizationStatus:
        status = LocalizationStatus()
        status.header = deepcopy(body.header)
        self.set_time_from_ns(status.last_vio_stamp, self.zed_odometry_stamp_ns)
        self.set_time_from_ns(status.last_tag_stamp, self.last_tag_stamp_ns)
        self.set_time_from_ns(
            status.last_absolute_fix_stamp, self.last_absolute_fix_stamp_ns
        )
        status.vio_age_s = float(self.age_s(now_ns, self.zed_odometry_stamp_ns))
        status.tag_age_s = float(self.age_s(now_ns, self.last_tag_stamp_ns))
        status.absolute_fix_age_s = float(
            self.age_s(now_ns, self.last_absolute_fix_stamp_ns)
        )
        status.apriltag_frame_age_s = float(
            self.age_s(
                now_ns,
                stamp_ns(self.apriltag_pose_status.header.stamp)
                if self.apriltag_pose_status is not None
                else 0,
            )
        )
        status.vio_rate_hz = float(self.rate_hz(self.vio_arrivals, now_ns))
        status.tag_rate_hz = float(self.rate_hz(self.tag_arrivals, now_ns))
        status.fused_rate_hz = float(self.rate_hz(self.fused_arrivals, now_ns))
        status.apriltag_frame_rate_hz = float(
            self.rate_hz(self.apriltag_frame_arrivals, now_ns)
        )
        status.vio_transport_delay_s = float(self.zed_transport_delay_s)
        status.tag_transport_delay_s = (
            float(self.last_tag_transport_delay_s)
        )
        status.tag_vio_translation_residual_m = float(translation_residual_m)
        status.tag_vio_angle_residual_deg = float(angle_residual_deg)
        status.pose_covariance = list(odometry.pose.covariance)
        status.twist_covariance = list(odometry.twist.covariance)
        status.detected_tag_count = int(self.detected_tag_count)
        if apriltag_status is not None:
            status.mapped_tag_count = int(apriltag_status.mapped_tag_count)
            status.inlier_tag_count = int(apriltag_status.inlier_tag_count)
            status.tag_reprojection_rms_px = float(
                apriltag_status.reprojection_rms_px
            )
            status.minimum_tag_edge_px = float(
                apriltag_status.minimum_tag_edge_px
            )
            status.tag_pose_published = bool(apriltag_status.pose_published)
            status.apriltag_rejection_reason = str(
                apriltag_status.rejection_reason
            )
        else:
            status.tag_reprojection_rms_px = math.nan
            status.minimum_tag_edge_px = math.nan
        status.vio_fresh = self.zed_odometry_is_fresh(now_ns)
        status.tag_fresh = tag is not None
        status.tag_consistent = bool(tag_consistent)
        status.absolute_fix_valid = bool(tag_accepted)
        status.position_estimated = bool(body.position_estimated)
        status.localization_source = str(body.localization_source)
        status.tag_observation_class = (
            tag.observation_class if tag is not None else ""
        )
        status.rejection_reason = str(rejection_reason)
        return status

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
