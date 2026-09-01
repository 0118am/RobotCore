#!/usr/bin/env python3
"""Capture and identify passive roll/pitch restoring dynamics from the IMU.

The script never publishes a ROS message. Capture is refused unless control
authority is disarmed and the latest thruster command is disabled and neutral.
The fitted free-decay model is

  theta_ddot = -(c/I) theta_dot -(q/I)|theta_dot|theta_dot
                -(K/I) sin(theta) + bias.

Without --effective-inertia-kg-m2 the identifiable normalized coefficients are
reported. With it, K is the 90-degree restoring torque in N m.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import numpy as np


@dataclass(frozen=True)
class Sample:
    stamp_ns: int
    wall_ns: int
    angle_rad: float
    rate_rad_s: float
    accel_x_m_s2: float
    accel_y_m_s2: float
    accel_z_m_s2: float


def quaternion_to_axis_angle(x: float, y: float, z: float, w: float, axis: str) -> float:
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(norm) or norm < 1.0e-9:
        raise ValueError("invalid IMU quaternion")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    if axis == "roll":
        return math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    return math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))


def _odd_window(duration_s: float, median_dt_s: float, sample_count: int) -> int:
    window = max(5, int(round(duration_s / median_dt_s)))
    if window % 2 == 0:
        window += 1
    maximum = sample_count - 2 if sample_count % 2 else sample_count - 3
    return max(3, min(window, maximum))


def fit_decay(
    time_s: Iterable[float],
    angle_rad: Iterable[float],
    rate_rad_s: Iterable[float],
    effective_inertia_kg_m2: float | None = None,
) -> dict[str, float | int | str | None]:
    """Fit normalized restoring and damping coefficients to one free decay."""
    t = np.asarray(list(time_s), dtype=float)
    angle = np.unwrap(np.asarray(list(angle_rad), dtype=float))
    rate = np.asarray(list(rate_rad_s), dtype=float)
    if t.size < 80 or angle.size != t.size or rate.size != t.size:
        raise ValueError("at least 80 equally sized time/angle/rate samples are required")
    finite = np.isfinite(t) & np.isfinite(angle) & np.isfinite(rate)
    t, angle, rate = t[finite], angle[finite], rate[finite]
    order = np.argsort(t)
    t, angle, rate = t[order], angle[order], rate[order]
    unique = np.concatenate(([True], np.diff(t) > 1.0e-6))
    t, angle, rate = t[unique], angle[unique], rate[unique]
    if t.size < 80:
        raise ValueError("too few distinct finite samples")
    dt = np.diff(t)
    median_dt = float(np.median(dt))
    if median_dt <= 0.0 or float(np.max(dt)) > max(0.1, 5.0 * median_dt):
        raise ValueError("sample timestamps contain a large gap")

    tail_start = t[-1] - min(2.0, 0.25 * (t[-1] - t[0]))
    equilibrium = float(np.median(angle[t >= tail_start]))
    theta = angle - equilibrium
    window = _odd_window(0.08, median_dt, t.size)
    kernel = np.ones(window, dtype=float) / float(window)
    smooth_rate = np.convolve(rate, kernel, mode="same")
    angular_acceleration = np.gradient(smooth_rate, t)
    edge = window // 2 + 1
    usable = np.zeros(t.size, dtype=bool)
    usable[edge:-edge] = True
    usable &= t >= t[0] + 0.10
    usable &= np.abs(theta) >= math.radians(2.0)
    if int(np.count_nonzero(usable)) < 50:
        raise ValueError("not enough moving free-decay data after filtering")

    omega = smooth_rate[usable]
    design = np.column_stack(
        (-omega, -np.abs(omega) * omega, -np.sin(theta[usable]), np.ones(omega.size))
    )
    response = angular_acceleration[usable]
    coefficients, _, rank, _ = np.linalg.lstsq(design, response, rcond=None)
    if rank < design.shape[1]:
        raise ValueError("free-decay data do not excite all fitted coefficients")
    linear_damping_over_inertia, quadratic_damping_over_inertia, restoring_over_inertia, bias = (
        float(value) for value in coefficients
    )
    prediction = design @ coefficients
    residual = response - prediction
    denominator = float(np.sum((response - np.mean(response)) ** 2))
    r_squared = 1.0 - float(np.sum(residual**2)) / denominator if denominator > 0.0 else math.nan
    natural_frequency = math.sqrt(restoring_over_inertia) if restoring_over_inertia > 0.0 else math.nan
    damping_ratio = (
        linear_damping_over_inertia / (2.0 * natural_frequency)
        if natural_frequency > 0.0
        else math.nan
    )
    result: dict[str, float | int | str | None] = {
        "model": "theta_ddot=-(c/I)omega-(q/I)|omega|omega-(K/I)sin(theta)+bias",
        "sample_count": int(t.size),
        "fit_sample_count": int(np.count_nonzero(usable)),
        "sample_rate_hz": 1.0 / median_dt,
        "equilibrium_angle_deg": math.degrees(equilibrium),
        "linear_damping_over_inertia_1_s": linear_damping_over_inertia,
        "quadratic_damping_over_inertia_1_rad": quadratic_damping_over_inertia,
        "restoring_over_inertia_rad_s2": restoring_over_inertia,
        "angular_acceleration_bias_rad_s2": bias,
        "natural_frequency_rad_s": natural_frequency,
        "linearized_damping_ratio": damping_ratio,
        "fit_r_squared": r_squared,
        "effective_inertia_kg_m2": effective_inertia_kg_m2,
        "linear_damping_n_m_s_per_rad": None,
        "quadratic_damping_n_m_s2_per_rad2": None,
        "restoring_torque_at_90_deg_n_m": None,
    }
    if effective_inertia_kg_m2 is not None:
        if not math.isfinite(effective_inertia_kg_m2) or effective_inertia_kg_m2 <= 0.0:
            raise ValueError("effective inertia must be positive")
        result["linear_damping_n_m_s_per_rad"] = (
            linear_damping_over_inertia * effective_inertia_kg_m2
        )
        result["quadratic_damping_n_m_s2_per_rad2"] = (
            quadratic_damping_over_inertia * effective_inertia_kg_m2
        )
        result["restoring_torque_at_90_deg_n_m"] = (
            restoring_over_inertia * effective_inertia_kg_m2
        )
    return result


def write_csv(path: Path, samples: list[Sample]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    first_stamp = samples[0].stamp_ns
    with path.open("w", newline="", encoding="utf-8") as output:
        writer = csv.writer(output)
        writer.writerow(
            ("time_s", "stamp_ns", "wall_ns", "angle_rad", "rate_rad_s",
             "accel_x_m_s2", "accel_y_m_s2", "accel_z_m_s2")
        )
        for sample in samples:
            writer.writerow(
                ((sample.stamp_ns - first_stamp) * 1.0e-9, sample.stamp_ns,
                 sample.wall_ns, sample.angle_rad, sample.rate_rad_s,
                 sample.accel_x_m_s2, sample.accel_y_m_s2, sample.accel_z_m_s2)
            )


def read_csv(path: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    with path.open(newline="", encoding="utf-8") as source:
        rows = list(csv.DictReader(source))
    return (
        np.asarray([float(row["time_s"]) for row in rows]),
        np.asarray([float(row["angle_rad"]) for row in rows]),
        np.asarray([float(row["rate_rad_s"]) for row in rows]),
    )


def capture(args: argparse.Namespace) -> list[Sample]:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from robotcore_interfaces.msg import ControlAuthorityStatus, ThrusterCommand
    from sensor_msgs.msg import Imu

    class CaptureNode(Node):
        def __init__(self) -> None:
            super().__init__("restoring_decay_capture")
            self.authority: ControlAuthorityStatus | None = None
            self.command: ThrusterCommand | None = None
            self.samples: list[Sample] = []
            self.baseline: list[float] = []
            self.safe_since: float | None = None
            self.held_since: float | None = None
            self.ready_announced = False
            self.trigger_stamp_ns: int | None = None
            self.failure: str | None = None
            self.done = threading.Event()
            self.create_subscription(
                ControlAuthorityStatus, "/control/authority/status", self.on_authority, 1
            )
            self.create_subscription(
                ThrusterCommand, "/control/thruster_cmd", self.on_command, 1
            )
            self.create_subscription(
                Imu,
                args.imu_topic,
                self.on_imu,
                QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
            )

        def safe(self) -> bool:
            return (
                self.authority is not None
                and not self.authority.armed
                and self.command is not None
                and not self.command.armed
                and not self.command.enable
                and max(abs(float(value)) for value in self.command.action) <= 0.01
            )

        def check_safety(self) -> None:
            if self.trigger_stamp_ns is not None and not self.safe():
                self.failure = "control became armed or thruster command became non-neutral"
                self.done.set()

        def on_authority(self, message: ControlAuthorityStatus) -> None:
            self.authority = message
            self.check_safety()

        def on_command(self, message: ThrusterCommand) -> None:
            self.command = message
            self.check_safety()

        def on_imu(self, message: Imu) -> None:
            if self.done.is_set():
                return
            now_mono = time.monotonic()
            if not self.safe():
                self.safe_since = None
                return
            if self.safe_since is None:
                self.safe_since = now_mono
                print("SAFE: authority disarmed and thrusters neutral; keep the robot upright briefly.")
                return
            if now_mono - self.safe_since < args.safe_hold_s:
                return
            try:
                angle = quaternion_to_axis_angle(
                    message.orientation.x, message.orientation.y,
                    message.orientation.z, message.orientation.w, args.axis
                )
            except ValueError as error:
                self.failure = str(error)
                self.done.set()
                return
            rate = float(
                message.angular_velocity.x if args.axis == "roll" else message.angular_velocity.y
            )
            stamp_ns = int(message.header.stamp.sec) * 1_000_000_000 + int(
                message.header.stamp.nanosec
            )
            if self.trigger_stamp_ns is None:
                if len(self.baseline) < 100:
                    self.baseline.append(angle)
                baseline = float(np.median(self.baseline)) if self.baseline else angle
                displacement = abs(math.atan2(math.sin(angle - baseline), math.cos(angle - baseline)))
                held = displacement >= math.radians(args.minimum_angle_deg) and abs(rate) <= args.hold_rate_rps
                if held:
                    self.held_since = self.held_since or now_mono
                    if now_mono - self.held_since >= args.angle_hold_s and not self.ready_announced:
                        self.ready_announced = True
                        print("ANGLE READY: release the robot now; recording starts on detected motion.")
                else:
                    self.held_since = None
                    self.ready_announced = False
                if self.ready_announced and abs(rate) >= args.release_rate_rps:
                    self.trigger_stamp_ns = stamp_ns
                    print(f"RELEASE DETECTED: collecting {args.duration_s:.1f} s of IMU data.")
                else:
                    return
            sample = Sample(
                stamp_ns, time.time_ns(), angle, rate,
                float(message.linear_acceleration.x),
                float(message.linear_acceleration.y),
                float(message.linear_acceleration.z),
            )
            self.samples.append(sample)
            if (stamp_ns - self.trigger_stamp_ns) * 1.0e-9 >= args.duration_s:
                self.done.set()

    rclpy.init()
    node = CaptureNode()
    print(
        "This tool is read-only. DISARM first, then place the robot upright; "
        f"it will wait for a {args.minimum_angle_deg:.0f} deg {args.axis} hold."
    )
    deadline = time.monotonic() + args.setup_timeout_s + args.duration_s
    try:
        while rclpy.ok() and not node.done.is_set() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    if node.failure:
        raise RuntimeError(node.failure)
    if not node.samples:
        raise RuntimeError("no release was captured before timeout")
    if (node.samples[-1].stamp_ns - node.samples[0].stamp_ns) * 1.0e-9 < 0.9 * args.duration_s:
        raise RuntimeError("capture ended before the requested duration")
    return node.samples


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--axis", choices=("roll", "pitch"), default="roll")
    parser.add_argument("--imu-topic", default="/sensors/external_imu")
    parser.add_argument("--duration-s", type=float, default=15.0)
    parser.add_argument("--minimum-angle-deg", type=float, default=60.0)
    parser.add_argument("--safe-hold-s", type=float, default=1.0)
    parser.add_argument("--angle-hold-s", type=float, default=0.7)
    parser.add_argument("--hold-rate-rps", type=float, default=0.10)
    parser.add_argument("--release-rate-rps", type=float, default=0.08)
    parser.add_argument("--setup-timeout-s", type=float, default=120.0)
    parser.add_argument("--effective-inertia-kg-m2", type=float)
    parser.add_argument("--output", type=Path, default=Path("restoring_decay.csv"))
    parser.add_argument("--analyze-only", type=Path)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        if args.analyze_only:
            time_s, angle, rate = read_csv(args.analyze_only)
            output = args.analyze_only
        else:
            samples = capture(args)
            write_csv(args.output, samples)
            time_s, angle, rate = read_csv(args.output)
            output = args.output
        result = fit_decay(time_s, angle, rate, args.effective_inertia_kg_m2)
        result["axis"] = args.axis
        summary_path = output.with_suffix(".json")
        summary_path.write_text(json.dumps(result, indent=2, allow_nan=True) + "\n", encoding="utf-8")
        print(json.dumps(result, indent=2, allow_nan=True))
        print(f"raw data: {output}\nfit summary: {summary_path}")
        if float(result["fit_r_squared"]) < 0.6:
            print("WARNING: weak fit; repeat the test and check for hand/tether contact.", file=sys.stderr)
        return 0
    except (RuntimeError, ValueError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
