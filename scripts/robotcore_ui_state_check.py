#!/usr/bin/env python3
"""Check browser-facing ControlInterface state for fresh camera and IMU data."""

from __future__ import annotations

import argparse
import json
import math
import sys
from datetime import datetime, timezone
from urllib import request


def parse_iso_utc(value: str) -> datetime | None:
    if not value:
        return None
    try:
        return datetime.fromisoformat(value.replace("Z", "+00:00"))
    except ValueError:
        return None


def age_seconds(value: str) -> float | None:
    stamp = parse_iso_utc(value)
    if stamp is None:
        return None
    if stamp.tzinfo is None:
        stamp = stamp.replace(tzinfo=timezone.utc)
    return (datetime.now(timezone.utc) - stamp.astimezone(timezone.utc)).total_seconds()


def quaternion_to_euler_degrees(values):
    if not isinstance(values, list) or len(values) < 4:
        raise ValueError("orientation must contain [w, x, y, z]")
    w, x, y, z = [float(item) for item in values[:4]]
    if not all(math.isfinite(item) for item in (w, x, y, z)):
        raise ValueError("orientation contains non-finite values")
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if not math.isfinite(norm) or norm <= 1e-9:
        raise ValueError("orientation quaternion norm is invalid")
    w, x, y, z = [item / norm for item in (w, x, y, z)]

    sinr = 2.0 * (w * x + y * z)
    cosr = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr, cosr)

    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)

    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny, cosy)
    scale = 180.0 / math.pi
    return roll * scale, pitch * scale, yaw * scale


def fetch_state(url: str) -> dict:
    with request.urlopen(url, timeout=2.0) as response:
        return json.loads(response.read().decode("utf-8"))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--url", default="http://127.0.0.1:8080/api/state")
    parser.add_argument("--max-camera-age-s", type=float, default=3.0)
    parser.add_argument("--require-orientation", action="store_true")
    args = parser.parse_args()

    try:
        state = fetch_state(args.url)
    except Exception as exc:
        print(f"ui state unavailable: {args.url}: {exc}", file=sys.stderr)
        return 1

    camera = state.get("camera") or {}
    if not camera.get("available"):
        print("camera unavailable in UI state", file=sys.stderr)
        return 1
    if int(camera.get("sequence") or 0) <= 0:
        print("camera has no received frame sequence", file=sys.stderr)
        return 1
    camera_age = age_seconds(str(camera.get("updated_at") or ""))
    if camera_age is None or camera_age > args.max_camera_age_s:
        print(f"camera frame is stale: age={camera_age}", file=sys.stderr)
        return 1

    imu = state.get("imu") or {}
    if not imu.get("available"):
        print("imu unavailable in UI state", file=sys.stderr)
        return 1
    angular = imu.get("angular_velocity")
    if not isinstance(angular, list) or len(angular) < 3 or not all(math.isfinite(float(v)) for v in angular[:3]):
        print("imu angular velocity invalid", file=sys.stderr)
        return 1
    try:
        roll, pitch, yaw = quaternion_to_euler_degrees(imu.get("orientation"))
    except ValueError as exc:
        if args.require_orientation:
            print(f"imu orientation invalid: {exc}", file=sys.stderr)
            return 1
        attitude = "orientation=unavailable"
    else:
        attitude = f"angular_xyz_deg={roll:.1f},{pitch:.1f},{yaw:.1f}"

    print(
        "ok ui_state "
        f"camera={camera.get('source') or 'n/a'} seq={camera.get('sequence')} age_s={camera_age:.2f} "
        f"angular_velocity_rad_s={float(angular[0]):.3f},{float(angular[1]):.3f},{float(angular[2]):.3f} "
        f"{attitude}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
