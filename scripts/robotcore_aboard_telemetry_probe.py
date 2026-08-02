#!/usr/bin/env python3
"""Measure A-board receive telemetry without transmitting motor commands."""

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

from eup_hardware.packet import (
    A_BOARD_PWM_FEEDBACK_HEADER,
    A_BOARD_PWM_FEEDBACK_LEN,
    A_BOARD_TELEMETRY_HEADER,
    A_BOARD_IMU_V1_TELEMETRY_LEN,
    A_BOARD_LEGACY_TELEMETRY_LEN,
    parse_uart_pwm_feedback_frame,
    parse_uart_telemetry_frame,
    telemetry_frame_length,
)


BAUD_CONSTANTS = {
    9600: termios.B9600,
    19200: termios.B19200,
    38400: termios.B38400,
    57600: termios.B57600,
    115200: termios.B115200,
}


def configure_read_only_port(fd: int, baud: int):
    if baud not in BAUD_CONSTANTS:
        raise ValueError(f"unsupported baud rate: {baud}")
    attributes = termios.tcgetattr(fd)
    configured = list(attributes)
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
    return attributes


def percentile(values: list[float], fraction: float) -> float:
    if not values:
        return math.nan
    ordered = sorted(values)
    index = max(0, min(len(ordered) - 1, math.ceil(fraction * len(ordered)) - 1))
    return ordered[index]


def next_frame(buffer: bytearray):
    starts = []
    for header in (A_BOARD_PWM_FEEDBACK_HEADER, A_BOARD_TELEMETRY_HEADER):
        index = buffer.find(header)
        if index >= 0:
            starts.append((index, header))
    if not starts:
        if len(buffer) > max(A_BOARD_PWM_FEEDBACK_LEN, A_BOARD_IMU_V1_TELEMETRY_LEN):
            del buffer[:-1]
        return None
    start, header = min(starts, key=lambda item: item[0])
    if start:
        del buffer[:start]
    if header == A_BOARD_PWM_FEEDBACK_HEADER:
        length = A_BOARD_PWM_FEEDBACK_LEN
    else:
        if len(buffer) < 3:
            return None
        length = telemetry_frame_length(buffer[2])
    if len(buffer) < length:
        return None
    frame = bytes(buffer[:length])
    return header, length, frame


def measure(port: str, baud: int, duration_s: float) -> int:
    fd = os.open(port, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
    original_attributes = None
    counters: Counter[str] = Counter()
    uart8_times: list[float] = []
    uart8_valid_times: list[float] = []
    buffer = bytearray()
    first_bad_frames: dict[int, str] = {}
    first_uart8_sample: dict[str, object] | None = None
    started = time.monotonic()
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        original_attributes = configure_read_only_port(fd, baud)
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
            counters["bytes"] += len(chunk)
            buffer.extend(chunk)
            while True:
                parsed_frame = next_frame(buffer)
                if parsed_frame is None:
                    break
                header, frame_length, frame = parsed_frame
                received_at = time.monotonic()
                try:
                    if header == A_BOARD_PWM_FEEDBACK_HEADER:
                        parse_uart_pwm_feedback_frame(frame)
                        counters["pwm_feedback"] += 1
                        del buffer[:frame_length]
                        continue
                    telemetry = parse_uart_telemetry_frame(frame)
                except ValueError:
                    counters["bad_frame"] += 1
                    frame_number = int(frame[2]) if len(frame) >= 3 else -1
                    counters[f"bad_telemetry_{frame_number}"] += 1
                    first_bad_frames.setdefault(frame_number, frame.hex(" "))
                    del buffer[:1]
                    continue
                del buffer[:frame_length]
                frame_number = int(frame[2])
                counters[f"telemetry_{frame_number}"] += 1
                if frame_number == 4:
                    if first_uart8_sample is None:
                        first_uart8_sample = telemetry
                    uart8_times.append(received_at)
                    if telemetry and telemetry.get("uart8_imu_valid"):
                        counters["uart8_valid"] += 1
                        uart8_valid_times.append(received_at)
                    else:
                        counters["uart8_invalid"] += 1
    finally:
        if original_attributes is not None:
            termios.tcsetattr(fd, termios.TCSANOW, original_attributes)
        os.close(fd)

    elapsed = max(1e-9, time.monotonic() - started)
    print(f"port={port} baud={baud} elapsed_s={elapsed:.3f}")
    for name in sorted(counters):
        count = counters[name]
        suffix = "" if name == "bytes" else f" rate_hz={count / elapsed:.3f}"
        print(f"{name}={count}{suffix}")
    for frame_number, frame_hex in sorted(first_bad_frames.items()):
        print(f"first_bad_telemetry_{frame_number}_hex={frame_hex}")
    if first_uart8_sample is not None:
        print(f"first_uart8_sample={first_uart8_sample}")
    if len(uart8_times) >= 2:
        intervals_ms = [
            (current - previous) * 1000.0
            for previous, current in zip(uart8_times, uart8_times[1:])
        ]
        source_span_s = uart8_times[-1] - uart8_times[0]
        print(f"uart8_span_rate_hz={(len(uart8_times) - 1) / source_span_s:.3f}")
        print(
            "uart8_arrival_interval_ms "
            f"mean={statistics.fmean(intervals_ms):.3f} "
            f"median={statistics.median(intervals_ms):.3f} "
            f"p95={percentile(intervals_ms, 0.95):.3f} "
            f"p99={percentile(intervals_ms, 0.99):.3f} "
            f"max={max(intervals_ms):.3f}"
        )
        zero_like = sum(interval <= 0.1 for interval in intervals_ms)
        print(f"uart8_batched_interval_le_0_1ms={zero_like}")
    if uart8_times and not uart8_valid_times:
        print("warning=no valid UART8 IMU samples observed")
        return 2
    if not uart8_times:
        print("warning=no versioned UART8 telemetry frame 4 observed")
        return 3
    return 0


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--port",
        default="/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B7A033320-if00",
    )
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--duration", type=float, default=10.0)
    arguments = parser.parse_args()
    return measure(arguments.port, arguments.baud, arguments.duration)


if __name__ == "__main__":
    raise SystemExit(main())
