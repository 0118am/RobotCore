#!/usr/bin/env python3
"""Read-only timing/integrity probe for Aquaboard UART protocol v2."""

from __future__ import annotations

import argparse
from collections import Counter
import fcntl
import math
import os
import statistics
import struct
import termios
import time


STATUS_HEADER = b"\xff\xfd"
IMU_HEADER = b"\xff\xf8\x04"
STATUS_LEN = 48
IMU_LEN = 27
PROTOCOL_V2 = 2
BAUD_CONSTANTS = {
    9600: termios.B9600,
    19200: termios.B19200,
    38400: termios.B38400,
    57600: termios.B57600,
    115200: termios.B115200,
}


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def configure_read_only_port(fd: int, baud: int):
    if baud not in BAUD_CONSTANTS:
        raise ValueError(f"unsupported baud rate: {baud}")
    original = termios.tcgetattr(fd)
    configured = list(original)
    configured[0] = 0
    configured[1] = 0
    configured[2] = termios.CLOCAL | termios.CREAD | termios.CS8
    configured[3] = 0
    configured[4] = BAUD_CONSTANTS[baud]
    configured[5] = BAUD_CONSTANTS[baud]
    configured[6] = list(configured[6])
    configured[6][termios.VMIN] = 0
    configured[6][termios.VTIME] = 1
    termios.tcsetattr(fd, termios.TCSANOW, configured)
    return original


def percentile(values: list[float], fraction: float) -> float:
    if not values:
        return math.nan
    ordered = sorted(values)
    index = max(0, min(len(ordered) - 1, math.ceil(fraction * len(ordered)) - 1))
    return ordered[index]


def next_frame(buffer: bytearray):
    starts = []
    for header, length, kind in (
        (STATUS_HEADER, STATUS_LEN, "status"),
        (IMU_HEADER, IMU_LEN, "imu"),
    ):
        index = buffer.find(header)
        if index >= 0:
            starts.append((index, length, kind))
    if not starts:
        if len(buffer) > STATUS_LEN:
            del buffer[:-2]
        return None
    start, length, kind = min(starts)
    if start:
        del buffer[:start]
    if len(buffer) < length:
        return None
    return kind, bytes(buffer[:length])


def require_crc(frame: bytes) -> None:
    if int.from_bytes(frame[-2:], "little") != crc16_ccitt(frame[:-2]):
        raise ValueError("CRC16 mismatch")


def parse_status(frame: bytes) -> dict[str, object]:
    if len(frame) != STATUS_LEN or frame[:2] != STATUS_HEADER or frame[2] != PROTOCOL_V2:
        raise ValueError("invalid status header/version")
    if frame[3] & ~0x07:
        raise ValueError("invalid status flags")
    require_crc(frame)
    tick, boot_id, session, received, applied = struct.unpack_from("<IIIII", frame, 4)
    age = struct.unpack_from("<H", frame, 24)[0]
    rx_crc_errors = struct.unpack_from("<H", frame, 28)[0]
    return {
        "flags": frame[3],
        "tick_ms": tick,
        "boot_id": boot_id,
        "session_id": session,
        "received_sequence": received,
        "applied_sequence": applied,
        "command_age_ms": age,
        "safety_reason": frame[26],
        "reset_cause": frame[27],
        "rx_crc_errors": rx_crc_errors,
        "pwm_us": list(struct.unpack_from("<8H", frame, 30)),
    }


def parse_imu(frame: bytes) -> dict[str, object]:
    if len(frame) != IMU_LEN or frame[:3] != IMU_HEADER or frame[3] != 1:
        raise ValueError("invalid IMU header/version")
    require_crc(frame)
    sample_id, tick_ms = struct.unpack_from("<II", frame, 5)
    values = struct.unpack_from("<6h", frame, 13)
    return {
        "valid": bool(frame[4] & 0x01),
        "sample_id": sample_id,
        "tick_ms": tick_ms,
        "gyro_cdeg_s": list(values[:3]),
        "accel_mg": list(values[3:]),
    }


def print_intervals(name: str, times: list[float]) -> None:
    if len(times) < 2:
        return
    intervals_ms = [
        (current - previous) * 1000.0
        for previous, current in zip(times, times[1:])
    ]
    span_s = times[-1] - times[0]
    print(f"{name}_span_rate_hz={(len(times) - 1) / span_s:.3f}")
    print(
        f"{name}_arrival_interval_ms "
        f"mean={statistics.fmean(intervals_ms):.3f} "
        f"median={statistics.median(intervals_ms):.3f} "
        f"p95={percentile(intervals_ms, 0.95):.3f} "
        f"p99={percentile(intervals_ms, 0.99):.3f} "
        f"max={max(intervals_ms):.3f}"
    )


def measure(port: str, baud: int, duration_s: float) -> int:
    fd = os.open(port, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
    original = None
    counts: Counter[str] = Counter()
    arrivals = {"status": [], "imu": []}
    first = {}
    buffer = bytearray()
    started = time.monotonic()
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        original = configure_read_only_port(fd, baud)
        deadline = started + max(0.1, duration_s)
        while time.monotonic() < deadline:
            try:
                chunk = os.read(fd, 4096)
            except BlockingIOError:
                time.sleep(0.0005)
                continue
            if not chunk:
                time.sleep(0.0005)
                continue
            counts["bytes"] += len(chunk)
            buffer.extend(chunk)
            while (candidate := next_frame(buffer)) is not None:
                kind, frame = candidate
                try:
                    parsed = parse_status(frame) if kind == "status" else parse_imu(frame)
                except ValueError:
                    counts[f"bad_{kind}"] += 1
                    del buffer[:1]
                    continue
                del buffer[: len(frame)]
                counts[kind] += 1
                arrivals[kind].append(time.monotonic())
                first.setdefault(kind, parsed)
                if kind == "imu" and not parsed["valid"]:
                    counts["imu_invalid"] += 1
    finally:
        if original is not None:
            termios.tcsetattr(fd, termios.TCSANOW, original)
        os.close(fd)

    elapsed = max(1e-9, time.monotonic() - started)
    print(f"port={port} baud={baud} elapsed_s={elapsed:.3f}")
    for name in sorted(counts):
        suffix = "" if name == "bytes" else f" rate_hz={counts[name] / elapsed:.3f}"
        print(f"{name}={counts[name]}{suffix}")
    for kind in ("status", "imu"):
        if kind in first:
            print(f"first_{kind}={first[kind]}")
        print_intervals(kind, arrivals[kind])
    if not arrivals["status"]:
        print("warning=no protocol-v2 board status observed")
        return 4
    if not arrivals["imu"]:
        print("warning=no versioned UART8 IMU frame observed")
        return 3
    if counts["imu_invalid"] == counts["imu"]:
        print("warning=no valid UART8 IMU samples observed")
        return 2
    return 0


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--port",
        default="/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B7A033320-if00",
    )
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--duration", type=float, default=10.0)
    args = parser.parse_args()
    return measure(args.port, args.baud, args.duration)


if __name__ == "__main__":
    raise SystemExit(main())
