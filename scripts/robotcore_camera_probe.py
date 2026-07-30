#!/usr/bin/env python3
"""Probe a local OpenCV/V4L2 camera and report whether one frame is readable."""

from __future__ import annotations

import argparse
import sys


def normalized_device(value: str):
    return int(value) if value.isdecimal() else value


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", default="/dev/video0", help="Camera device path or numeric OpenCV index.")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    args = parser.parse_args()

    try:
        import cv2
    except Exception as exc:
        print(f"cv2 unavailable: {exc}", file=sys.stderr)
        return 2

    capture = cv2.VideoCapture(normalized_device(args.device))
    try:
        if not capture.isOpened():
            print(f"camera unavailable: {args.device}", file=sys.stderr)
            return 1

        if args.width > 0:
            capture.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
        if args.height > 0:
            capture.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)

        ok, frame = capture.read()
        if not ok or frame is None:
            print(f"camera opened but no frame was readable: {args.device}", file=sys.stderr)
            return 1

        shape = getattr(frame, "shape", ())
        if len(shape) < 2:
            print(f"camera returned unsupported frame shape: {shape}", file=sys.stderr)
            return 1

        print(f"ok {args.device} {int(shape[1])}x{int(shape[0])}")
        return 0
    finally:
        capture.release()


if __name__ == "__main__":
    raise SystemExit(main())
