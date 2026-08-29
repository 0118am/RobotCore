"""Observation state machine for the t60_precision_v7 trajectory policy.

Pose and linear velocity come from RobotCore BodyState, angular velocity comes
from the base_link external IMU, and previous action comes from the canonical
post-authority thruster command without an actuator adapter. All use the same
right-handed FLU convention: +X forward, +Y left, +Z up.
"""

from __future__ import annotations

import numpy as np


OBSERVATION_LAYOUT = "t60_precision_v7_history8"

OBS_SCALE = np.asarray(
    [
        0.25,
        0.25,
        0.25,
        0.40,
        0.40,
        0.40,
        0.20,
        0.20,
        0.20,
        1.00,
        1.00,
        1.00,
        1.00,
        1.00,
        1.00,
        1.00,
        0.80,
        0.80,
        0.80,
        0.80,
        0.80,
        0.80,
        0.45,
        0.45,
        0.45,
        1.00,
        1.00,
        1.00,
        1.00,
        1.00,
        1.00,
        1.00,
        1.00,
    ],
    dtype=np.float32,
)

HISTORY_INDICES = np.r_[0:3, 6:13, 16:19, 25:33]


class T60ObservationState:
    """Build the exact 33 + 8*21 observation expected by model_499.

    History is committed only after a successful inference. The command in each
    frame is the canonical post-authority command accepted by the vehicle.
    """

    current_dim = 33
    history_length = 8
    history_sample_dim = 21
    observation_dim = 201
    action_dim = 8

    def __init__(self):
        self.history = np.zeros(
            (self.history_length, self.history_sample_dim), dtype=np.float32
        )
        self._pending_obs33 = None

    def reset(self):
        self.history.fill(0.0)
        self._pending_obs33 = None

    def build(self, observation):
        """Return a contiguous float32 array with shape ``(1, 201)``."""

        obs33 = self.build_current_frame(observation)
        policy_input = np.concatenate((obs33, self.history.reshape(-1))).astype(
            np.float32, copy=False
        )
        if policy_input.shape != (self.observation_dim,):
            raise RuntimeError(
                f"t60 observation has shape {policy_input.shape}, expected "
                f"({self.observation_dim},)"
            )
        if not np.all(np.isfinite(policy_input)):
            raise RuntimeError("t60 observation contains a non-finite value")
        self._pending_obs33 = obs33.copy()
        return np.ascontiguousarray(policy_input.reshape(1, -1))

    def commit(self):
        """Commit the just-inferred observation frame to newest-first history."""

        if self._pending_obs33 is None:
            raise RuntimeError("cannot commit t60 history before building an observation")
        self.history[1:] = self.history[:-1].copy()
        self.history[0] = self._pending_obs33[HISTORY_INDICES]
        self._pending_obs33 = None

    def build_current_frame(self, observation):
        body = observation.get("/robot/body_state", {})
        external_imu = observation.get("/sensors/external_imu", {})
        thruster_command = observation.get("/control/thruster_cmd", {})
        target = observation.get("/runtime/trajectory_target", {})
        if not body:
            raise RuntimeError("t60 observation is missing /robot/body_state")
        if not target or not target.get("valid", False):
            raise RuntimeError("t60 observation requires a valid trajectory target")

        body_pose = body.get("pose", {})
        body_twist = body.get("twist", {})
        target_pose = target.get("pose", {})
        target_twist = target.get("twist", {})
        target_accel = target.get("accel", {})

        measured_position_w = self._vec3(body_pose.get("position"), "body position")
        localization_quaternion_w = self._unit_quaternion(
            body_pose.get("orientation"), "body orientation"
        )
        imu_quaternion_w = self._unit_quaternion(
            external_imu.get("orientation"), "external IMU orientation"
        )
        localization_rpy = self._quaternion_to_rpy(localization_quaternion_w)
        imu_rpy = self._quaternion_to_rpy(imu_quaternion_w)
        measured_quaternion_w = self._rpy_quaternion(
            np.asarray(
                [imu_rpy[0], imu_rpy[1], localization_rpy[2]],
                dtype=np.float32,
            )
        )
        measured_linear_velocity_b = self._vec3(
            body_twist.get("linear"), "body linear velocity"
        )
        measured_angular_velocity_b = self._vec3(
            external_imu.get("angular_velocity"), "external IMU angular velocity"
        )
        previous_command = np.asarray(
            thruster_command["action"], dtype=np.float32
        ).reshape(self.action_dim)

        target_position_w = self._vec3(
            target_pose.get("position"), "target position"
        )
        target_quaternion_w = self._unit_quaternion(
            target_pose.get("orientation"), "target orientation"
        )
        target_linear_velocity_w = self._vec3(
            target_twist.get("linear"), "target linear velocity"
        )
        target_angular_velocity_w = self._vec3(
            target_twist.get("angular"), "target angular velocity"
        )
        target_linear_acceleration_w = self._vec3(
            target_accel.get("linear"), "target linear acceleration"
        )

        q_bw = self._quat_conjugate(measured_quaternion_w)
        position_error_b = self._quat_apply(
            q_bw, target_position_w - measured_position_w
        )
        target_linear_velocity_b = self._quat_apply(
            q_bw, target_linear_velocity_w
        )
        linear_velocity_error_b = (
            target_linear_velocity_b - measured_linear_velocity_b
        )
        attitude_error_quat = self._quat_unique(
            self._quat_multiply(q_bw, target_quaternion_w)
        )
        projected_gravity_b = self._quat_apply(
            q_bw, np.asarray([0.0, 0.0, -1.0], dtype=np.float32)
        )
        target_angular_velocity_b = self._quat_apply(
            q_bw, target_angular_velocity_w
        )
        target_linear_acceleration_b = self._quat_apply(
            q_bw, target_linear_acceleration_w
        )

        raw_obs33 = np.concatenate(
            (
                position_error_b,
                target_linear_velocity_b,
                linear_velocity_error_b,
                attitude_error_quat,
                projected_gravity_b,
                measured_angular_velocity_b,
                target_angular_velocity_b,
                target_linear_acceleration_b,
                previous_command,
            )
        ).astype(np.float32, copy=False)
        if raw_obs33.shape != (self.current_dim,):
            raise RuntimeError(
                f"t60 current observation has shape {raw_obs33.shape}, expected "
                f"({self.current_dim},)"
            )
        return raw_obs33 / OBS_SCALE

    @staticmethod
    def _vec3(values, label):
        array = np.asarray(values, dtype=np.float32).reshape(-1)
        if array.shape != (3,) or not np.all(np.isfinite(array)):
            raise RuntimeError(f"{label} must contain three finite values")
        return array

    @staticmethod
    def _unit_quaternion(values, label):
        quaternion = np.asarray(values, dtype=np.float32).reshape(-1)
        if quaternion.shape != (4,) or not np.all(np.isfinite(quaternion)):
            raise RuntimeError(f"{label} must be a finite wxyz quaternion")
        norm = float(np.linalg.norm(quaternion))
        if norm < 1.0e-6:
            raise RuntimeError(f"{label} quaternion norm is zero")
        return quaternion / np.float32(norm)

    @staticmethod
    def _quat_conjugate(quaternion):
        return quaternion * np.asarray([1.0, -1.0, -1.0, -1.0], dtype=np.float32)

    @staticmethod
    def _quat_multiply(left, right):
        lw, lx, ly, lz = left
        rw, rx, ry, rz = right
        return np.asarray(
            [
                lw * rw - lx * rx - ly * ry - lz * rz,
                lw * rx + lx * rw + ly * rz - lz * ry,
                lw * ry - lx * rz + ly * rw + lz * rx,
                lw * rz + lx * ry - ly * rx + lz * rw,
            ],
            dtype=np.float32,
        )

    @staticmethod
    def _quat_unique(quaternion):
        return -quaternion if quaternion[0] < 0.0 else quaternion

    @staticmethod
    def _quat_apply(quaternion, vector):
        xyz = quaternion[1:4]
        cross = 2.0 * np.cross(xyz, vector)
        return (
            vector + quaternion[0] * cross + np.cross(xyz, cross)
        ).astype(np.float32)

    @staticmethod
    def _quaternion_to_rpy(quaternion):
        w, x, y, z = quaternion
        return np.asarray(
            [
                np.arctan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y)),
                np.arcsin(np.clip(2.0 * (w * y - z * x), -1.0, 1.0)),
                np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)),
            ],
            dtype=np.float32,
        )

    @staticmethod
    def _rpy_quaternion(rpy):
        roll, pitch, yaw = rpy
        cr, sr = np.cos(roll * 0.5), np.sin(roll * 0.5)
        cp, sp = np.cos(pitch * 0.5), np.sin(pitch * 0.5)
        cy, sy = np.cos(yaw * 0.5), np.sin(yaw * 0.5)
        quaternion = np.asarray(
            [
                cr * cp * cy + sr * sp * sy,
                sr * cp * cy - cr * sp * sy,
                cr * sp * cy + sr * cp * sy,
                cr * cp * sy - sr * sp * cy,
            ],
            dtype=np.float32,
        )
        return quaternion / np.linalg.norm(quaternion)
