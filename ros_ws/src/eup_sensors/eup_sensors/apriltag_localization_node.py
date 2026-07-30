"""Estimate ``map -> base_link`` from calibrated camera AprilTag observations."""

from __future__ import annotations

from collections import deque
import json
import math
from pathlib import Path
import threading
import time
from typing import Any

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import Int32
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

from .localization_math import (
    advance_rate_deadline,
    invert_transform,
    largest_tag_quads,
    per_tag_full_corner_rms_px,
    quaternion_xyzw,
    rotation_from_quaternion_xyzw,
    slerp_quaternion_xyzw,
    tag_ids_with_inlier_corner_count,
    tag36h11_corners_in_map_axis_order,
    tag_corners_in_map,
    transform_matrix,
    validate_cuboid_pool_tag_layout,
)
from .tag_vio_alignment_math import interpolate_transform, rotation_distance_rad


class AprilTagLocalizationNode(Node):
    """Fuse all visible mapped tags into one PnP pose for the robot body."""

    def __init__(self):
        super().__init__("apriltag_localization")
        self.declare_parameter("image_topic", "/zedx/zed_node/rgb/color/rect/image")
        self.declare_parameter("camera_info_topic", "/zedx/zed_node/rgb/color/rect/camera_info")
        # The default image is rectified. Its projection must use CameraInfo.P,
        # not the raw-image K/D calibration pair.
        self.declare_parameter("image_is_rectified", True)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("tag_family", "tag36h11")
        self.declare_parameter("tag_size_m", 0.13)
        self.declare_parameter("tag_map_file", "")
        # The managed map describes a 5.42 m F by 3.73 m L rectangular pool
        # whose origin is the lower-right-rear corner. Reject a complete JSON
        # revision if any Tag is off the floor/walls, faces out of the pool,
        # is upside down on a wall, or extends outside those bounds.
        self.declare_parameter("enforce_cuboid_pool_geometry", True)
        self.declare_parameter("pool_length_m", 5.42)
        self.declare_parameter("pool_width_m", 3.73)
        self.declare_parameter("pool_surface_tolerance_m", 0.02)
        self.declare_parameter("pool_orientation_tolerance_deg", 2.0)
        # The host manager writes tag maps atomically.  Polling the file's
        # mtime lets an approved map edit take effect without restarting the
        # localization graph. Set zero to require a restart instead.
        self.declare_parameter("tag_map_reload_interval_s", 1.0)
        self.declare_parameter("relocalize_service", "/localization/apriltag/relocalize")
        self.declare_parameter("pose_topic", "/localization/apriltag_pose")
        # Strict two-Tag observations use a separate topic. The map-to-odom
        # alignment node deliberately never subscribes to it, so degraded
        # observations cannot establish or recalibrate the global transform.
        self.declare_parameter(
            "degraded_pose_topic", "/localization/apriltag_pose_degraded"
        )
        self.declare_parameter(
            "aligned_vio_odometry_topic", "/localization/fused_odom"
        )
        # This reports raw visual detections, including IDs that do not yet
        # exist in the map.  It is intentionally independent of PnP success
        # so the operator can finish mapping while localisation is incomplete.
        self.declare_parameter("detected_count_topic", "/localization/apriltag/detected_count")
        self.declare_parameter("debug_image_topic", "/localization/apriltag/debug_image/compressed")
        self.declare_parameter("debug_jpeg_quality", 85)
        # The operator browser receives a latest-only BEST_EFFORT stream.
        # Drawing and JPEG encoding run on a dedicated worker so they cannot
        # halve the localisation callback rate when a browser is connected.
        self.declare_parameter("debug_publish_rate_hz", 15.0)
        self.declare_parameter("debug_max_width_px", 960)
        self.declare_parameter("publish_tf", True)
        # Measured camera optical centre relative to the vehicle centre of
        # mass (base_link), expressed in ROS FLU coordinates.
        self.declare_parameter("base_to_camera_translation_m", [0.236, 0.027, 0.016])
        self.declare_parameter(
            "base_to_camera_optical_rpy_rad", [-math.pi / 2.0, 0.0, -math.pi / 2.0]
        )
        self.declare_parameter("max_reprojection_error_px", 4.0)
        # Measured tag centres in a large tank are not millimetre-accurate.
        # Convert this world-space allowance to pixels from each observed
        # tag's apparent size instead of using an unsafe fixed pixel limit.
        self.declare_parameter("tag_map_position_uncertainty_m", 0.05)
        # Publish the quality-gated PnP observation by default.  A non-zero
        # value can smooth measured jitter, but it also adds motion lag.
        self.declare_parameter("pose_filter_time_constant_s", 0.0)
        # Keep filtering through a brief tag loss. Reinitialising from the
        # first reacquired single-tag PnP sample makes the displayed distance
        # visibly jump.
        self.declare_parameter("pose_filter_reset_after_s", 0.0)
        self.declare_parameter("pose_filter_max_elapsed_s", 0.20)
        self.declare_parameter("min_tag_edge_px", 20.0)
        self.declare_parameter("max_reprojection_rms_px", 3.0)
        # A raw detection is not a global-position correction.  Require at
        # least three mapped tags in the same frame before Tag data can move
        # the VIO-aligned map pose.
        self.declare_parameter("minimum_pose_tag_count", 3)
        self.declare_parameter("minimum_inlier_corners_per_tag", 3)
        # Once map->odom exists, exactly two fully supported Tags may validate
        # its propagated VIO pose. This path never updates the primary pose
        # filter or its reacquisition state.
        self.declare_parameter("enable_degraded_two_tag_pose", True)
        self.declare_parameter("degraded_two_tag_inlier_corners_per_tag", 4)
        self.declare_parameter("degraded_two_tag_max_full_rms_px", 3.0)
        self.declare_parameter("degraded_two_tag_vio_sync_tolerance_s", 0.12)
        self.declare_parameter("degraded_two_tag_vio_history_s", 3.0)
        self.declare_parameter("degraded_two_tag_vio_max_translation_m", 0.20)
        self.declare_parameter("degraded_two_tag_vio_max_angle_deg", 10.0)
        self.declare_parameter("degraded_two_tag_position_stddev_m", 0.10)
        self.declare_parameter("degraded_two_tag_angle_stddev_deg", 5.0)
        # Only quality-gated visual observations are allowed to calibrate the
        # global VIO frame. Operators can still inspect raw detections through
        # detected_count and the debug image when a PnP observation is rejected.
        self.declare_parameter("enforce_observation_gates", True)
        self.declare_parameter("enforce_transition_gate", True)
        self.declare_parameter("min_tag_depth_m", 0.10)
        # A small fixed allowance rejects a bad planar-PnP solution, while the
        # 1 m/s term still permits the robot's commanded motion at camera rate.
        self.declare_parameter("max_translation_speed_mps", 1.0)
        self.declare_parameter("max_translation_jump_m", 0.05)
        # A delayed frame must not widen the innovation gate enough for a
        # competing planar-tag PnP solution to be accepted as real motion.
        self.declare_parameter("pose_transition_max_elapsed_s", 0.25)
        # A real robot can be repositioned farther than the per-frame jump
        # gate permits.  Do not freeze localisation forever in that case:
        # accept a new pose only after several mutually consistent, already
        # quality-checked AprilTag observations.
        self.declare_parameter("reacquisition_confirm_frames", 4)
        self.declare_parameter("reacquisition_max_spread_m", 0.12)
        self.declare_parameter("reacquisition_max_gap_s", 0.50)
        # These are conservative defaults for the accepted (three-or-more
        # Tag) PnP observation, not a substitute for a calibrated covariance.
        self.declare_parameter("multi_tag_position_stddev_m", 0.05)
        self.declare_parameter("multi_tag_angle_stddev_deg", 2.0)

        self.cv2 = self.require_cv2()
        tag_map_filename = str(self.get_parameter("tag_map_file").value).strip()
        self.tag_map_path = Path(tag_map_filename).expanduser() if tag_map_filename else None
        self.tag_layout, _ = self.load_tag_layout(tag_map_filename)
        self.tag_map_observed_mtime_ns = self.tag_map_mtime_ns()
        self.dictionary = self.make_dictionary(str(self.get_parameter("tag_family").value))
        self.detector_parameters = self.make_detector_parameters()
        self.detector = self.make_detector()
        self.camera_matrix: np.ndarray | None = None
        self.distortion: np.ndarray | None = None
        self.camera_info_size: tuple[int, int] | None = None
        self.last_error = ""
        self.last_warning_at_s: dict[str, float] = {}
        self.last_accepted_inlier_tag_ids: tuple[int, ...] = ()
        self.last_accepted_degraded_tag_ids: tuple[int, ...] = ()
        self.filtered_map_from_base: np.ndarray | None = None
        self.filtered_pose_stamp_ns = 0
        self.aligned_vio_history: deque[tuple[int, np.ndarray]] = deque()
        # An operator-requested relocalization declares the previous global
        # pose untrustworthy.  The next geometrically valid observation must
        # therefore not be compared with that old pose by the transition gate.
        self.relocalization_pending = False
        self.reacquisition_candidate: np.ndarray | None = None
        self.reacquisition_stamp_ns = 0
        self.reacquisition_count = 0
        self.next_debug_publish_at_s = float("-inf")
        self._debug_condition = threading.Condition()
        self._debug_pending: tuple[Image, Any, Any] | None = None
        self._debug_stopping = False
        self.base_to_camera = transform_matrix(
            self.get_parameter("base_to_camera_translation_m").value,
            self.get_parameter("base_to_camera_optical_rpy_rad").value,
        )

        # AprilTag localisation is a real-time consumer: a queued old image is
        # worse than a dropped one.  Use one best-effort sample throughout the
        # image path so detection, overlay generation, and the browser stay on
        # the newest camera frame.
        self.latest_image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, str(self.get_parameter("pose_topic").value), 10
        )
        self.degraded_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            str(self.get_parameter("degraded_pose_topic").value),
            10,
        )
        self.detected_count_pub = self.create_publisher(
            Int32, str(self.get_parameter("detected_count_topic").value), 10
        )
        # This image is intended for the operator UI. It retains the source
        # timestamp, while throttled, downscaled JPEG encoding keeps it out of
        # the localisation critical path.
        self.debug_image_pub = self.create_publisher(
            CompressedImage,
            str(self.get_parameter("debug_image_topic").value),
            self.latest_image_qos,
        )
        self.tf_broadcaster = (
            TransformBroadcaster(self) if bool(self.get_parameter("publish_tf").value) else None
        )
        self.create_subscription(
            CameraInfo,
            str(self.get_parameter("camera_info_topic").value),
            self.on_camera_info,
            self.latest_image_qos,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter("image_topic").value),
            self.on_image,
            self.latest_image_qos,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("aligned_vio_odometry_topic").value),
            self.on_aligned_vio_odometry,
            20,
        )
        self.relocalize_service = self.create_service(
            Trigger,
            str(self.get_parameter("relocalize_service").value),
            self.on_relocalize,
        )
        reload_interval_s = float(self.get_parameter("tag_map_reload_interval_s").value)
        self.tag_map_reload_timer = (
            self.create_timer(reload_interval_s, self.reload_tag_layout_if_changed)
            if reload_interval_s > 0.0
            else None
        )
        self._debug_thread = threading.Thread(
            target=self._debug_worker,
            name="apriltag-debug-jpeg",
            daemon=True,
        )
        self._debug_thread.start()
        self.get_logger().info(
            f"AprilTag localisation listens on {self.get_parameter('image_topic').value}; "
            f"mapped tags: {sorted(self.tag_layout)}"
        )

    def require_cv2(self):
        try:
            import cv2
        except Exception as exc:
            raise RuntimeError(f"OpenCV with aruco support is required: {exc}") from exc
        if not hasattr(cv2, "aruco"):
            raise RuntimeError("OpenCV was built without the aruco/AprilTag module")
        return cv2

    def make_dictionary(self, family: str):
        if family.lower() != "tag36h11":
            raise ValueError(
                f"unsupported AprilTag family: {family}; "
                "this localization pipeline is calibrated for tag36h11"
            )
        constant = "DICT_APRILTAG_36h11"
        if not hasattr(self.cv2.aruco, constant):
            raise RuntimeError("OpenCV was built without tag36h11 support")
        return self.cv2.aruco.getPredefinedDictionary(getattr(self.cv2.aruco, constant))

    def make_detector_parameters(self):
        if hasattr(self.cv2.aruco, "DetectorParameters_create"):
            return self.cv2.aruco.DetectorParameters_create()
        return self.cv2.aruco.DetectorParameters()

    def make_detector(self):
        if hasattr(self.cv2.aruco, "ArucoDetector"):
            return self.cv2.aruco.ArucoDetector(self.dictionary, self.detector_parameters)
        return None

    def load_tag_layout(
        self, filename: str
    ) -> tuple[dict[int, dict[str, np.ndarray]], bool]:
        """Load a complete map and report whether parsing succeeded.

        An empty ``tags`` object is a valid map after the final definition has
        been deleted. It must be distinguishable from a malformed or missing
        map so reload can clear stale localisation geometry safely.
        """

        if not filename:
            self.get_logger().warn("tag_map_file is empty; localisation will not publish poses")
            return {}, False
        path = Path(filename).expanduser()
        try:
            if not path.is_file():
                self.get_logger().warn(f"tag map file does not exist: {path}; localisation will not publish poses")
                return {}, False
            if path.suffix.lower() != ".json":
                raise ValueError("tag_map_file must be a JSON map")
            data = json.loads(path.read_text(encoding="utf-8")) or {}
            if not isinstance(data, dict):
                raise ValueError("tag map root must be a JSON object")
            tags = data.get("tags", {})
            if not isinstance(tags, dict):
                raise ValueError("tag map tags must be a JSON object")
            layout: dict[int, dict[str, np.ndarray]] = {}
            default_size = float(self.get_parameter("tag_size_m").value)
            if not math.isfinite(default_size) or default_size <= 0.0:
                raise ValueError("tag_size_m must be a positive finite value")
            for raw_id, definition in tags.items():
                center = np.asarray(definition["position_m"], dtype=np.float64)
                rpy = np.deg2rad(np.asarray(definition["rpy_deg"], dtype=np.float64))
                size = float(definition.get("size_m", default_size))
                if (
                    center.shape != (3,)
                    or rpy.shape != (3,)
                    or not np.isfinite(center).all()
                    or not np.isfinite(rpy).all()
                    or not math.isfinite(size)
                    or size <= 0.0
                ):
                    raise ValueError(f"tag {raw_id} must contain finite three-value position_m and rpy_deg")
                layout[int(raw_id)] = {"position_m": center, "rpy_rad": rpy, "size_m": size}
            if layout and bool(
                self.get_parameter("enforce_cuboid_pool_geometry").value
            ):
                validate_cuboid_pool_tag_layout(
                    layout,
                    pool_length_m=float(self.get_parameter("pool_length_m").value),
                    pool_width_m=float(self.get_parameter("pool_width_m").value),
                    surface_tolerance_m=float(
                        self.get_parameter("pool_surface_tolerance_m").value
                    ),
                    orientation_tolerance_deg=float(
                        self.get_parameter("pool_orientation_tolerance_deg").value
                    ),
                )
            if not layout:
                self.get_logger().warn(f"tag map file {path} has no tags; localisation will not publish poses")
            return layout, True
        except OSError as exc:
            self.get_logger().warn(f"cannot read tag map {path}: {exc}; localisation will not publish poses")
            return {}, False
        except Exception as exc:
            self.get_logger().error(f"failed to load tag map {path}: {exc}")
            return {}, False

    def tag_map_mtime_ns(self) -> int | None:
        """Return the active map timestamp without treating a missing file as valid."""

        try:
            if self.tag_map_path is None:
                return None
            return self.tag_map_path.stat().st_mtime_ns
        except OSError:
            return None

    def reload_tag_layout_if_changed(self):
        """Adopt only a complete, valid, atomically written map revision."""

        observed_mtime_ns = self.tag_map_mtime_ns()
        if observed_mtime_ns == self.tag_map_observed_mtime_ns:
            return
        self.tag_map_observed_mtime_ns = observed_mtime_ns
        if observed_mtime_ns is None:
            if self.tag_map_path is not None:
                self.get_logger().error("AprilTag map disappeared; preserving the last valid layout")
            return
        replacement, loaded = self.load_tag_layout(str(self.tag_map_path))
        if not loaded:
            self.get_logger().error("AprilTag map reload rejected; preserving the last valid layout")
            return
        self.tag_layout = replacement
        self.reset_pose_filter()
        self.get_logger().info(f"AprilTag map reloaded: mapped tags {sorted(self.tag_layout)}")

    def on_relocalize(self, _request: Trigger.Request, response: Trigger.Response):
        """Reload the map now and accept the next valid AprilTag pose fresh."""

        if self.tag_map_path is None:
            response.success = False
            response.message = "AprilTag map path is not configured"
            return response

        replacement, loaded = self.load_tag_layout(str(self.tag_map_path))
        if not loaded:
            response.success = False
            response.message = "AprilTag map reload rejected; preserving the current localization"
            return response

        self.tag_layout = replacement
        self.tag_map_observed_mtime_ns = self.tag_map_mtime_ns()
        self.reset_pose_filter()
        self.aligned_vio_history.clear()
        self.relocalization_pending = True
        response.success = True
        response.message = (
            "AprilTag map reloaded; pose filter reset; the next valid pose will "
            f"bypass the old-pose jump gate; mapped tags {sorted(self.tag_layout)}"
        )
        self.get_logger().info(response.message)
        return response

    def on_camera_info(self, msg: CameraInfo):
        if bool(self.get_parameter("image_is_rectified").value):
            # CameraInfo.K/D describe the raw, distorted image. P describes
            # the processed rectified image subscribed above.
            matrix = np.asarray(msg.p, dtype=np.float64).reshape(3, 4)[:, :3]
            distortion = np.zeros((5, 1), dtype=np.float64)
        else:
            matrix = np.asarray(msg.k, dtype=np.float64).reshape(3, 3)
            distortion = np.asarray(msg.d, dtype=np.float64).reshape(-1, 1)
        if (
            not np.isfinite(matrix).all()
            or matrix[0, 0] <= 0.0
            or matrix[1, 1] <= 0.0
            or int(msg.width) <= 0
            or int(msg.height) <= 0
        ):
            self.warn_once("camera_info has invalid focal lengths")
            return
        self.camera_matrix = matrix
        self.distortion = distortion
        self.camera_info_size = (int(msg.width), int(msg.height))

    @staticmethod
    def message_stamp_ns(message) -> int:
        return int(message.header.stamp.sec) * 1_000_000_000 + int(
            message.header.stamp.nanosec
        )

    def on_aligned_vio_odometry(self, message: Odometry):
        """Keep map-frame VIO predictions only after map->odom is established."""

        map_frame = str(self.get_parameter("map_frame").value)
        base_frame = str(self.get_parameter("base_frame").value)
        if message.header.frame_id != map_frame or message.child_frame_id != base_frame:
            self.warn_throttled(
                "degraded-vio-frame",
                f"aligned VIO must be {map_frame}->{base_frame}; "
                f"received {message.header.frame_id}->{message.child_frame_id}",
            )
            return
        try:
            pose = message.pose.pose
            map_from_base = np.eye(4, dtype=np.float64)
            map_from_base[:3, :3] = rotation_from_quaternion_xyzw(
                [
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w,
                ]
            )
            map_from_base[:3, 3] = [
                float(pose.position.x),
                float(pose.position.y),
                float(pose.position.z),
            ]
            if not np.isfinite(map_from_base).all():
                raise ValueError("pose contains non-finite values")
        except (TypeError, ValueError) as exc:
            self.warn_throttled(
                "degraded-vio-pose", f"aligned VIO pose is invalid: {exc}"
            )
            return
        sample_ns = self.message_stamp_ns(message)
        if sample_ns <= 0:
            return
        if self.aligned_vio_history and sample_ns < self.aligned_vio_history[-1][0]:
            self.warn_throttled(
                "degraded-vio-time", "aligned VIO timestamp moved backwards"
            )
            return
        sample = (sample_ns, map_from_base)
        if self.aligned_vio_history and sample_ns == self.aligned_vio_history[-1][0]:
            self.aligned_vio_history[-1] = sample
        else:
            self.aligned_vio_history.append(sample)
        history_ns = int(
            max(
                0.1,
                float(
                    self.get_parameter("degraded_two_tag_vio_history_s").value
                ),
            )
            * 1e9
        )
        while (
            self.aligned_vio_history
            and sample_ns - self.aligned_vio_history[0][0] > history_ns
        ):
            self.aligned_vio_history.popleft()

    def aligned_vio_pose_at(self, target_ns: int) -> np.ndarray | None:
        """Interpolate an already map-aligned VIO pose at an image timestamp."""

        if not self.aligned_vio_history:
            return None
        tolerance_ns = int(
            max(
                0.0,
                float(
                    self.get_parameter(
                        "degraded_two_tag_vio_sync_tolerance_s"
                    ).value
                ),
            )
            * 1e9
        )
        before = None
        after = None
        for sample in self.aligned_vio_history:
            if sample[0] <= target_ns:
                before = sample
            if sample[0] >= target_ns:
                after = sample
                break
        if before is None or after is None:
            nearest = (
                self.aligned_vio_history[0]
                if before is None
                else self.aligned_vio_history[-1]
            )
            return nearest[1] if abs(nearest[0] - target_ns) <= tolerance_ns else None
        if before[0] == after[0]:
            return before[1]
        if max(target_ns - before[0], after[0] - target_ns) > tolerance_ns:
            return None
        fraction = (target_ns - before[0]) / (after[0] - before[0])
        return interpolate_transform(before[1], after[1], fraction)

    def on_image(self, msg: Image):
        corners = None
        ids = None
        try:
            gray = self.image_to_gray(msg)
            if self.detector is not None:
                corners, ids, _ = self.detector.detectMarkers(gray)
            else:
                corners, ids, _ = self.cv2.aruco.detectMarkers(
                    gray, self.dictionary, parameters=self.detector_parameters
                )
            if self.camera_matrix is None or self.distortion is None:
                self.warn_once("waiting for calibrated CameraInfo before AprilTag localisation")
                return
            if self.camera_info_size != (int(msg.width), int(msg.height)):
                self.warn_once("CameraInfo/image dimensions differ; refusing invalid AprilTag PnP geometry")
                return
            if not self.tag_layout:
                self.warn_once("waiting for measured tag IDs and map poses in tag_map_file")
                return
            object_points, image_points, seen_ids = self.mapped_correspondences(corners, ids)
            if len(seen_ids) == 0:
                return
            minimum_tag_count = max(
                3, int(self.get_parameter("minimum_pose_tag_count").value)
            )
            degraded_two_tag = (
                bool(self.get_parameter("enable_degraded_two_tag_pose").value)
                and len(seen_ids) == 2
            )
            if len(seen_ids) < minimum_tag_count and not degraded_two_tag:
                self.warn_throttled(
                    "too-few-mapped-tags",
                    "AprilTag pose withheld: "
                    f"mapped Tags {seen_ids}, require at least {minimum_tag_count}",
                )
                return
            success, rvec, tvec, inliers = self.solve_tag_pnp(
                object_points, image_points, seen_ids
            )
            if not success or inliers is None or len(inliers) < 4:
                self.warn_throttled(
                    "pnp-rejected",
                    "multi-tag PnP rejected for mapped Tags "
                    f"{seen_ids}; withholding Tag correction",
                )
                return
            minimum_inlier_corners = (
                int(
                    self.get_parameter(
                        "degraded_two_tag_inlier_corners_per_tag"
                    ).value
                )
                if degraded_two_tag
                else int(
                    self.get_parameter("minimum_inlier_corners_per_tag").value
                )
            )
            inlier_tag_ids = self.inlier_tag_ids(
                inliers, seen_ids, minimum_corners=minimum_inlier_corners
            )
            required_supported_tags = 2 if degraded_two_tag else minimum_tag_count
            if len(inlier_tag_ids) < required_supported_tags:
                self.warn_throttled(
                    "pnp-inlier-tags",
                    "multi-tag PnP lacks support from at least "
                    f"{required_supported_tags} Tags with "
                    f"{minimum_inlier_corners}/4 inlier corners each "
                    f"(inliers: {inlier_tag_ids}); "
                    "withholding Tag correction",
                )
                return
            rotation, _ = self.cv2.Rodrigues(rvec)
            enforce_observation_gates = bool(
                self.get_parameter("enforce_observation_gates").value
            )
            observation_is_credible = (
                self.pnp_observation_is_credible(
                    object_points, image_points, seen_ids, inliers, rvec, rotation, tvec
                )
                if enforce_observation_gates or degraded_two_tag
                else True
            )
            if (enforce_observation_gates or degraded_two_tag) and not observation_is_credible:
                return
            camera_from_map = np.eye(4, dtype=np.float64)
            camera_from_map[:3, :3] = rotation
            camera_from_map[:3, 3] = tvec.reshape(3)
            map_from_camera = invert_transform(camera_from_map)
            map_from_base = map_from_camera @ invert_transform(self.base_to_camera)
            if degraded_two_tag:
                self.publish_degraded_two_tag_pose(
                    msg,
                    map_from_base,
                    object_points,
                    image_points,
                    seen_ids,
                    inlier_tag_ids,
                    rvec,
                    tvec,
                )
                return
            if self.relocalization_pending:
                # Relocalize means the old global pose is explicitly invalid.
                # Keep the observation/PnP quality gates above, but accept any
                # translation or rotation change relative to the old pose.
                self.relocalization_pending = False
                self.clear_reacquisition_candidate()
            elif bool(
                self.get_parameter("enforce_transition_gate").value
            ) and not self.pose_transition_is_credible(
                map_from_base, msg, seen_ids, inlier_tag_ids
            ):
                return
            map_from_base = self.filter_map_from_base(map_from_base, msg)
            accepted_inlier_tag_ids = tuple(inlier_tag_ids)
            if accepted_inlier_tag_ids != self.last_accepted_inlier_tag_ids:
                self.get_logger().info(
                    "AprilTag correction accepted from mapped Tags "
                    f"{seen_ids}; PnP inlier Tags {inlier_tag_ids}"
                )
                self.last_accepted_inlier_tag_ids = accepted_inlier_tag_ids
            self.publish_pose(msg, map_from_base)
            self.last_error = ""
        except Exception as exc:
            self.warn_throttled(
                "frame-rejected", f"AprilTag localisation frame rejected: {exc}"
            )
        finally:
            # Publish after each processed image, even when the camera model,
            # map, or PnP result is not usable.  A zero is therefore a real
            # “no tag in this frame”, while no message means no camera input.
            self.publish_detected_count(ids)
            # The operator overlay is diagnostic-only. Queue only the newest
            # frame; drawing/JPEG work must never hold up the next detection.
            self.queue_debug_image(msg, corners, ids)

    def publish_detected_count(self, ids):
        """Publish the number of raw IDs recognized in the current image."""

        message = Int32()
        message.data = 0 if ids is None else int(len(ids))
        self.detected_count_pub.publish(message)

    def solve_tag_pnp(self, object_points, image_points, seen_ids: list[int]):
        """Solve the joint PnP observation from the mapped Tags in one frame."""

        reprojection_limit_px = max(
            0.0, float(self.get_parameter("max_reprojection_error_px").value)
        ) + self.tag_map_uncertainty_allowance_px(image_points, seen_ids)
        return self.cv2.solvePnPRansac(
            object_points,
            image_points,
            self.camera_matrix,
            self.distortion,
            flags=self.cv2.SOLVEPNP_ITERATIVE,
            reprojectionError=reprojection_limit_px,
            confidence=0.999,
            iterationsCount=100,
        )

    def pnp_observation_is_credible(
        self, object_points, image_points, seen_ids, inliers, rvec, rotation, tvec
    ) -> bool:
        """Reject weak planar-tag PnP solutions before they reach the pose filter."""

        tag_points = np.asarray(image_points, dtype=np.float64).reshape(-1, 4, 2)
        side_lengths = np.linalg.norm(tag_points - np.roll(tag_points, -1, axis=1), axis=2)
        minimum_edge_px = max(0.0, float(self.get_parameter("min_tag_edge_px").value))
        small_tag_ids = [
            int(tag_id)
            for tag_id, lengths in zip(seen_ids, side_lengths)
            if float(np.min(lengths)) < minimum_edge_px
        ]
        if small_tag_ids:
            self.warn_throttled(
                "small-tag",
                "AprilTag observation is too small for stable distance estimation: "
                f"mapped Tags {small_tag_ids}",
            )
            return False

        camera_points = (
            rotation @ np.asarray(object_points, dtype=np.float64).T
        ).T + tvec.reshape(1, 3)
        minimum_depth_m = max(0.0, float(self.get_parameter("min_tag_depth_m").value))
        tag_camera_points = camera_points.reshape(-1, 4, 3)
        invalid_depth_ids = [
            int(tag_id)
            for tag_id, points in zip(seen_ids, tag_camera_points)
            if float(np.min(points[:, 2])) <= minimum_depth_m
        ]
        if invalid_depth_ids:
            self.warn_throttled(
                "invalid-tag-depth",
                "AprilTag PnP placed mapped Tags "
                f"{invalid_depth_ids} behind or too close to the camera",
            )
            return False

        projected, _ = self.cv2.projectPoints(
            object_points, rvec, tvec, self.camera_matrix, self.distortion
        )
        residuals = np.asarray(image_points, dtype=np.float64) - projected.reshape(-1, 2)
        selected = residuals[np.asarray(inliers, dtype=np.intp).reshape(-1)]
        rms_px = float(math.sqrt(np.mean(np.sum(selected * selected, axis=1))))
        rms_limit_px = max(
            0.0, float(self.get_parameter("max_reprojection_rms_px").value)
        ) + self.tag_map_uncertainty_allowance_px(image_points, seen_ids)
        if not math.isfinite(rms_px) or rms_px > rms_limit_px:
            inlier_indices = np.asarray(inliers, dtype=np.intp).reshape(-1)
            per_tag_rms_px = {}
            for tag_index, tag_id in enumerate(seen_ids):
                start = 4 * tag_index
                tag_inliers = inlier_indices[
                    (inlier_indices >= start) & (inlier_indices < start + 4)
                ]
                if len(tag_inliers):
                    tag_residuals = residuals[tag_inliers]
                    per_tag_rms_px[int(tag_id)] = round(
                        float(
                            math.sqrt(
                                np.mean(np.sum(tag_residuals * tag_residuals, axis=1))
                            )
                        ),
                        2,
                    )
            self.warn_throttled(
                "reprojection-rms",
                "AprilTag PnP reprojection RMS "
                f"{rms_px:.2f}px exceeds {rms_limit_px:.2f}px limit; "
                f"per-Tag inlier RMS(px) {per_tag_rms_px}",
            )
            return False
        return True

    def publish_degraded_two_tag_pose(
        self,
        image: Image,
        observed: np.ndarray,
        object_points,
        image_points,
        seen_ids: list[int],
        inlier_tag_ids: list[int],
        rvec,
        tvec,
    ):
        """Publish a two-Tag VIO validation without touching alignment state."""

        if self.relocalization_pending:
            self.warn_throttled(
                "degraded-relocalization",
                "AprilTag two-Tag pose withheld during relocalization; "
                "three Tags are required",
            )
            return
        if sorted(inlier_tag_ids) != sorted(seen_ids):
            return

        projected, _ = self.cv2.projectPoints(
            object_points, rvec, tvec, self.camera_matrix, self.distortion
        )
        residuals = np.asarray(image_points, dtype=np.float64) - projected.reshape(-1, 2)
        per_tag_rms_px = per_tag_full_corner_rms_px(residuals, seen_ids)
        maximum_rms_px = max(
            0.0,
            float(
                self.get_parameter("degraded_two_tag_max_full_rms_px").value
            ),
        )
        if any(
            not math.isfinite(rms_px) or rms_px > maximum_rms_px
            for rms_px in per_tag_rms_px.values()
        ):
            rounded = {
                tag_id: round(rms_px, 2)
                for tag_id, rms_px in per_tag_rms_px.items()
            }
            self.warn_throttled(
                "degraded-full-rms",
                "AprilTag two-Tag pose withheld: full-corner RMS(px) "
                f"{rounded} exceeds {maximum_rms_px:.2f}px",
            )
            return

        predicted = self.aligned_vio_pose_at(self.message_stamp_ns(image))
        if predicted is None:
            self.warn_throttled(
                "degraded-no-vio",
                "AprilTag two-Tag pose withheld: no time-aligned map-frame "
                "VIO prediction; three Tags are required to establish alignment",
            )
            return
        translation_residual_m = float(
            np.linalg.norm(observed[:3, 3] - predicted[:3, 3])
        )
        angle_residual_deg = math.degrees(
            rotation_distance_rad(observed, predicted)
        )
        maximum_translation_m = max(
            0.0,
            float(
                self.get_parameter(
                    "degraded_two_tag_vio_max_translation_m"
                ).value
            ),
        )
        maximum_angle_deg = max(
            0.0,
            float(
                self.get_parameter("degraded_two_tag_vio_max_angle_deg").value
            ),
        )
        if (
            translation_residual_m > maximum_translation_m
            or angle_residual_deg > maximum_angle_deg
        ):
            self.warn_throttled(
                "degraded-vio-disagreement",
                "AprilTag two-Tag pose withheld: disagrees with aligned VIO by "
                f"{translation_residual_m:.2f}m/{angle_residual_deg:.1f}deg; "
                f"limits are {maximum_translation_m:.2f}m/{maximum_angle_deg:.1f}deg",
            )
            return

        accepted_ids = tuple(sorted(inlier_tag_ids))
        if accepted_ids != self.last_accepted_degraded_tag_ids:
            rounded = {
                tag_id: round(rms_px, 2)
                for tag_id, rms_px in per_tag_rms_px.items()
            }
            self.get_logger().info(
                "AprilTag degraded two-Tag VIO validation accepted from Tags "
                f"{list(accepted_ids)}; full-corner RMS(px) {rounded}; "
                f"VIO residual {translation_residual_m:.2f}m/"
                f"{angle_residual_deg:.1f}deg"
            )
            self.last_accepted_degraded_tag_ids = accepted_ids
        self.publish_pose(image, observed, degraded=True)
        self.last_error = ""

    def tag_map_uncertainty_allowance_px(self, image_points, seen_ids: list[int]) -> float:
        """Project the configured tag-centre uncertainty into the current image."""

        uncertainty_m = max(
            0.0, float(self.get_parameter("tag_map_position_uncertainty_m").value)
        )
        if uncertainty_m == 0.0 or not seen_ids:
            return 0.0

        tag_points = np.asarray(image_points, dtype=np.float64).reshape(-1, 4, 2)
        allowances_px = []
        for points, tag_id in zip(tag_points, seen_ids):
            tag_size_m = float(self.tag_layout[tag_id]["size_m"])
            side_lengths_px = np.linalg.norm(points - np.roll(points, -1, axis=0), axis=1)
            allowances_px.append(
                uncertainty_m * float(np.max(side_lengths_px)) / tag_size_m
            )
        return max(allowances_px, default=0.0)

    def pose_transition_is_credible(
        self,
        observed: np.ndarray,
        image: Image,
        seen_ids: list[int],
        inlier_tag_ids: list[int],
    ) -> bool:
        """Reject one-off jumps, but reinitialise after a stable new observation."""

        if self.filtered_map_from_base is None:
            self.clear_reacquisition_candidate()
            return True
        stamp_ns = int(image.header.stamp.sec) * 1_000_000_000 + int(image.header.stamp.nanosec)
        elapsed_s = (stamp_ns - self.filtered_pose_stamp_ns) / 1e9
        if elapsed_s <= 0.0:
            self.warn_throttled(
                "image-timestamp",
                "AprilTag image timestamp is not strictly increasing",
            )
            return False
        maximum_gate_elapsed_s = max(
            0.0, float(self.get_parameter("pose_transition_max_elapsed_s").value)
        )
        if maximum_gate_elapsed_s > 0.0:
            elapsed_s = min(elapsed_s, maximum_gate_elapsed_s)
        allowed_m = max(0.0, float(self.get_parameter("max_translation_jump_m").value)) + (
            max(0.0, float(self.get_parameter("max_translation_speed_mps").value)) * elapsed_s
        )
        displacement_m = float(
            np.linalg.norm(np.asarray(observed[:3, 3]) - np.asarray(self.filtered_map_from_base[:3, 3]))
        )
        if displacement_m > allowed_m:
            self.warn_throttled(
                "pose-transition",
                f"AprilTag pose jump {displacement_m:.2f}m exceeds "
                f"{allowed_m:.2f}m limit; mapped Tags {seen_ids}, "
                f"PnP inlier Tags {inlier_tag_ids}",
            )
            return self.confirm_reacquisition(observed, stamp_ns)
        self.clear_reacquisition_candidate()
        return True

    def confirm_reacquisition(self, observed: np.ndarray, stamp_ns: int) -> bool:
        """Accept a stable pose cluster after the normal transition gate rejects it."""

        maximum_gap_ns = int(
            max(0.0, float(self.get_parameter("reacquisition_max_gap_s").value)) * 1e9
        )
        maximum_spread_m = max(
            0.0, float(self.get_parameter("reacquisition_max_spread_m").value)
        )
        candidate = self.reacquisition_candidate
        sample_is_consistent = (
            candidate is not None
            and stamp_ns > self.reacquisition_stamp_ns
            and (maximum_gap_ns == 0 or stamp_ns - self.reacquisition_stamp_ns <= maximum_gap_ns)
            and float(
                np.linalg.norm(
                    np.asarray(observed[:3, 3]) - np.asarray(candidate[:3, 3])
                )
            ) <= maximum_spread_m
        )
        if sample_is_consistent:
            self.reacquisition_count += 1
        else:
            self.reacquisition_candidate = np.asarray(observed, dtype=np.float64).copy()
            self.reacquisition_count = 1
        self.reacquisition_stamp_ns = stamp_ns

        required_samples = max(
            1, int(self.get_parameter("reacquisition_confirm_frames").value)
        )
        if self.reacquisition_count < required_samples:
            return False

        self.get_logger().warn(
            "AprilTag accepted a stable pose after transition-gate rejection; "
            "resetting the pose filter"
        )
        self.reset_pose_filter()
        return True

    def reset_pose_filter(self):
        """Discard geometry derived from the previous tag-map revision."""

        self.filtered_map_from_base = None
        self.filtered_pose_stamp_ns = 0
        self.last_accepted_inlier_tag_ids = ()
        self.last_accepted_degraded_tag_ids = ()
        self.clear_reacquisition_candidate()

    def clear_reacquisition_candidate(self):
        self.reacquisition_candidate = None
        self.reacquisition_stamp_ns = 0
        self.reacquisition_count = 0

    def filter_map_from_base(self, observed: np.ndarray, image: Image) -> np.ndarray:
        """Apply a timestamp-aware low-pass to direct AprilTag map poses."""

        stamp_ns = int(image.header.stamp.sec) * 1_000_000_000 + int(image.header.stamp.nanosec)
        time_constant_s = max(0.0, float(self.get_parameter("pose_filter_time_constant_s").value))
        reset_after_s = max(0.0, float(self.get_parameter("pose_filter_reset_after_s").value))
        elapsed_s = (stamp_ns - self.filtered_pose_stamp_ns) / 1e9
        should_reset = (
            self.filtered_map_from_base is None
            or stamp_ns <= self.filtered_pose_stamp_ns
            or (reset_after_s > 0.0 and elapsed_s > reset_after_s)
        )
        if should_reset or time_constant_s == 0.0:
            filtered = np.asarray(observed, dtype=np.float64).copy()
        else:
            maximum_elapsed_s = max(
                0.0, float(self.get_parameter("pose_filter_max_elapsed_s").value)
            )
            if maximum_elapsed_s > 0.0:
                elapsed_s = min(elapsed_s, maximum_elapsed_s)
            alpha = 1.0 - math.exp(-elapsed_s / time_constant_s)
            previous = self.filtered_map_from_base
            previous_q = np.asarray(quaternion_xyzw(previous[:3, :3]), dtype=np.float64)
            observed_q = np.asarray(quaternion_xyzw(observed[:3, :3]), dtype=np.float64)
            filtered_q = slerp_quaternion_xyzw(previous_q, observed_q, alpha)
            filtered = np.eye(4, dtype=np.float64)
            filtered[:3, :3] = rotation_from_quaternion_xyzw(filtered_q)
            filtered[:3, 3] = (1.0 - alpha) * previous[:3, 3] + alpha * observed[:3, 3]
        self.filtered_map_from_base = filtered
        self.filtered_pose_stamp_ns = stamp_ns
        return filtered

    def queue_debug_image(self, source: Image, corners, ids):
        """Replace the pending operator frame without blocking localisation."""

        if self.debug_image_pub.get_subscription_count() == 0:
            return
        debug_rate_hz = float(self.get_parameter("debug_publish_rate_hz").value)
        if debug_rate_hz <= 0.0:
            return
        now_s = time.monotonic()
        interval_s = 1.0 / debug_rate_hz
        if now_s < self.next_debug_publish_at_s:
            return
        # Preserve fractional rate phase. Resetting the deadline to
        # ``now + interval`` would quantize 20 Hz detection to 10 Hz when the
        # requested operator rate is 15 Hz.
        self.next_debug_publish_at_s = advance_rate_deadline(
            now_s, self.next_debug_publish_at_s, interval_s
        )
        with self._debug_condition:
            self._debug_pending = (source, corners, ids)
            self._debug_condition.notify()

    def _debug_worker(self):
        """Encode latest-only diagnostic frames outside the image callback."""

        while True:
            with self._debug_condition:
                self._debug_condition.wait_for(
                    lambda: self._debug_stopping or self._debug_pending is not None
                )
                if self._debug_stopping:
                    return
                pending = self._debug_pending
                self._debug_pending = None
            if pending is None:
                continue
            try:
                self.publish_debug_image(*pending)
            except Exception as exc:
                self.warn_throttled(
                    "debug-image-rejected", f"AprilTag debug image rejected: {exc}"
                )

    def publish_debug_image(self, source: Image, corners, ids):
        """Draw, encode, and publish one queued AprilTag operator frame."""

        if self.debug_image_pub.get_subscription_count() == 0:
            return
        image = self.image_to_bgr(source)
        if ids is not None and len(ids):
            self.cv2.aruco.drawDetectedMarkers(image, corners, ids, borderColor=(0, 255, 0))
            for tag_id, detected_corners in zip(ids.reshape(-1), corners):
                if int(tag_id) in self.tag_layout:
                    continue
                center = np.asarray(detected_corners, dtype=np.float64).reshape(4, 2).mean(axis=0)
                self.cv2.putText(
                    image,
                    "not mapped",
                    (int(center[0]), int(center[1]) + 20),
                    self.cv2.FONT_HERSHEY_SIMPLEX,
                    0.45,
                    (0, 180, 255),
                    1,
                    self.cv2.LINE_AA,
                )
        maximum_width_px = max(0, int(self.get_parameter("debug_max_width_px").value))
        if maximum_width_px and image.shape[1] > maximum_width_px:
            scale = maximum_width_px / image.shape[1]
            image = self.cv2.resize(
                image,
                (maximum_width_px, max(1, round(image.shape[0] * scale))),
                interpolation=self.cv2.INTER_AREA,
            )
        quality = max(1, min(100, int(self.get_parameter("debug_jpeg_quality").value)))
        success, encoded = self.cv2.imencode(
            ".jpg", image, [int(self.cv2.IMWRITE_JPEG_QUALITY), quality]
        )
        if not success:
            self.warn_once("failed to JPEG-compress AprilTag debug frame")
            return
        output = CompressedImage()
        output.header = source.header
        output.format = "jpeg"
        output.data = encoded.tobytes()
        self.debug_image_pub.publish(output)

    def mapped_correspondences(self, corners, ids):
        object_points: list[np.ndarray] = []
        image_points: list[np.ndarray] = []
        seen_ids: list[int] = []
        if ids is None:
            return np.empty((0, 3)), np.empty((0, 2)), seen_ids
        # One ID maps to one physical pose. A false duplicate decode (or two
        # physical copies of one ID) must not add incompatible PnP points.
        for tag_id, detected_corners in largest_tag_quads(
            corners, ids, self.tag_layout
        ):
            definition = self.tag_layout[tag_id]
            object_points.extend(
                tag_corners_in_map(definition["position_m"], definition["rpy_rad"], definition["size_m"])
            )
            image_points.extend(tag36h11_corners_in_map_axis_order(detected_corners))
            seen_ids.append(tag_id)
        return np.asarray(object_points, dtype=np.float64), np.asarray(image_points, dtype=np.float64), seen_ids

    @staticmethod
    def inlier_tag_ids(
        inliers, seen_ids: list[int], minimum_corners: int = 3
    ) -> list[int]:
        """Return Tags supported by several corners, never a lone RANSAC corner."""

        return tag_ids_with_inlier_corner_count(
            inliers, seen_ids, minimum_corners
        )

    def image_to_gray(self, msg: Image) -> np.ndarray:
        encoding = msg.encoding.lower()
        channels = {"mono8": 1, "8uc1": 1, "bgr8": 3, "rgb8": 3, "bgra8": 4, "rgba8": 4}.get(encoding)
        if channels is None:
            raise ValueError(f"unsupported image encoding {msg.encoding}; use mono8, bgr8, rgb8, bgra8, or rgba8")
        row = np.frombuffer(msg.data, dtype=np.uint8).reshape(int(msg.height), int(msg.step))
        pixels = row[:, : int(msg.width) * channels]
        if channels == 1:
            return pixels.reshape(int(msg.height), int(msg.width))
        image = pixels.reshape(int(msg.height), int(msg.width), channels)
        conversion = {
            "bgr8": self.cv2.COLOR_BGR2GRAY,
            "rgb8": self.cv2.COLOR_RGB2GRAY,
            "bgra8": self.cv2.COLOR_BGRA2GRAY,
            "rgba8": self.cv2.COLOR_RGBA2GRAY,
        }[encoding]
        return self.cv2.cvtColor(image, conversion)

    def image_to_bgr(self, msg: Image) -> np.ndarray:
        """Decode a supported ROS image into a drawable BGR image."""

        encoding = msg.encoding.lower()
        channels = {"mono8": 1, "8uc1": 1, "bgr8": 3, "rgb8": 3, "bgra8": 4, "rgba8": 4}.get(encoding)
        if channels is None:
            raise ValueError(f"unsupported image encoding {msg.encoding}; use mono8, bgr8, rgb8, bgra8, or rgba8")
        row = np.frombuffer(msg.data, dtype=np.uint8).reshape(int(msg.height), int(msg.step))
        pixels = row[:, : int(msg.width) * channels]
        if encoding == "bgr8":
            return pixels.reshape(int(msg.height), int(msg.width), 3).copy()
        image = pixels.reshape(int(msg.height), int(msg.width), channels)
        conversion = {
            "mono8": self.cv2.COLOR_GRAY2BGR,
            "8uc1": self.cv2.COLOR_GRAY2BGR,
            "rgb8": self.cv2.COLOR_RGB2BGR,
            "bgra8": self.cv2.COLOR_BGRA2BGR,
            "rgba8": self.cv2.COLOR_RGBA2BGR,
        }[encoding]
        return self.cv2.cvtColor(image, conversion)

    def publish_pose(
        self, image: Image, map_from_base: np.ndarray, *, degraded: bool = False
    ):
        x, y, z, w = quaternion_xyzw(map_from_base[:3, :3])
        pose = PoseWithCovarianceStamped()
        pose.header.stamp = image.header.stamp
        pose.header.frame_id = str(self.get_parameter("map_frame").value)
        pose.pose.pose.position.x = float(map_from_base[0, 3])
        pose.pose.pose.position.y = float(map_from_base[1, 3])
        pose.pose.pose.position.z = float(map_from_base[2, 3])
        pose.pose.pose.orientation.x = x
        pose.pose.pose.orientation.y = y
        pose.pose.pose.orientation.z = z
        pose.pose.pose.orientation.w = w
        # Values are conservative defaults, not a substitute for calibrated
        # covariance. The degraded topic uses larger uncertainty and is never
        # consumed by map-to-odom alignment.
        position_stddev = float(
            self.get_parameter(
                "degraded_two_tag_position_stddev_m"
                if degraded
                else "multi_tag_position_stddev_m"
            ).value
        )
        angle_stddev_deg = float(
            self.get_parameter(
                "degraded_two_tag_angle_stddev_deg"
                if degraded
                else "multi_tag_angle_stddev_deg"
            ).value
        )
        position_variance = max(0.001, position_stddev) ** 2
        angle_variance = math.radians(max(0.1, angle_stddev_deg)) ** 2
        for index in (0, 7, 14):
            pose.pose.covariance[index] = position_variance
        for index in (21, 28, 35):
            pose.pose.covariance[index] = angle_variance
        (self.degraded_pose_pub if degraded else self.pose_pub).publish(pose)

        # In the fused configuration Tag/VIO alignment owns the map-frame
        # pose. Broadcasting this raw
        # AprilTag pose as map->base_link at the same time would create a
        # competing transform authority, so the launch file disables it.
        if self.tf_broadcaster is not None and not degraded:
            transform = TransformStamped()
            transform.header = pose.header
            transform.child_frame_id = str(self.get_parameter("base_frame").value)
            transform.transform.translation = pose.pose.pose.position
            transform.transform.rotation = pose.pose.pose.orientation
            self.tf_broadcaster.sendTransform(transform)

    def warn_once(self, message: str):
        if message != self.last_error:
            self.get_logger().warn(message)
            self.last_error = message

    def warn_throttled(self, key: str, message: str, interval_s: float = 5.0):
        """Log a changing frame-level rejection at a bounded rate."""

        now_s = time.monotonic()
        if now_s - self.last_warning_at_s.get(key, float("-inf")) < interval_s:
            return
        self.last_warning_at_s[key] = now_s
        self.get_logger().warn(message)

    def destroy_node(self):
        with self._debug_condition:
            self._debug_stopping = True
            self._debug_pending = None
            self._debug_condition.notify()
        self._debug_thread.join(timeout=2.0)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagLocalizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
