"""Tests for the Aboard packet helper contract."""

import math
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
# Import directly from the source package so packet tests do not require a ROS 2
# build/install step.
sys.path.insert(0, str(ROOT / "ros_ws/src/eup_hardware"))

from eup_hardware.packet import (
    build_uart_direct_pwm_frame,
    normalized_to_direct_pwm_offsets,
    parse_uart_telemetry_frame,
    parse_uart_pwm_feedback_frame,
)


def test_uart_direct_pwm_frame_has_expected_layout():
    frame = build_uart_direct_pwm_frame([20, -20])
    assert frame[:2] == bytes([0xFF, 0xFA])
    assert len(frame) == 35
    assert frame[2:6] == bytes([20, 0, 236, 255])
    assert frame[-1] == (sum(frame[:-1]) & 0xFF)


def test_uart_feedback_frame_parses_sixteen_pwm_readings():
    payload = b"".join(int(1500 + i).to_bytes(2, "little") for i in range(16))
    frame = bytearray([0xFF, 0xFB])
    frame.extend(payload)
    frame.append(sum(frame) & 0xFF)
    assert parse_uart_pwm_feedback_frame(bytes(frame)) == [1500 + i for i in range(16)]


def test_uart_telemetry_frame_zero_parses_aboard_imu_values():
    values = [10, -20, 30, 1234, -50, 100, -200, 300]
    frame = bytearray([0xFF, 0xF8, 0])
    for value in values:
        frame.extend(int(value).to_bytes(2, "little", signed=True))
    frame.append(sum(frame) & 0xFF)

    telemetry = parse_uart_telemetry_frame(bytes(frame))
    assert telemetry["pitch_deg"] == 10.0
    assert telemetry["roll_deg"] == -20.0
    assert telemetry["yaw_deg"] == 30.0
    assert telemetry["depth_m"] == 1.234
    assert telemetry["gyro_y_dps"] == 1.0
    assert telemetry["gyro_x_dps"] == -2.0
    assert telemetry["gyro_z_dps"] == 3.0


def test_uart_telemetry_frame_two_parses_acceleration_mps2():
    values = [4096, -4096, 2048, 0, 0, 0, 0, 0]
    frame = bytearray([0xFF, 0xF8, 2])
    for value in values:
        frame.extend(int(value).to_bytes(2, "little", signed=True))
    frame.append(sum(frame) & 0xFF)

    telemetry = parse_uart_telemetry_frame(bytes(frame))
    assert telemetry["accel_x_mps2"] == 9.80665
    assert telemetry["accel_y_mps2"] == -9.80665
    # Both legacy and UART8 paths now expose raw specific force. Gravity is
    # removed once, after bias/attitude calibration, by eup_sensors.
    assert telemetry["accel_z_mps2"] == 4.903325


def test_uart_telemetry_frame_three_parses_aboard_uart8_imu():
    values = [125, -250, 50, 100, -200, 981, 1, 0]
    frame = bytearray([0xFF, 0xF8, 3])
    for value in values:
        frame.extend(int(value).to_bytes(2, "little", signed=True))
    frame.append(sum(frame) & 0xFF)

    telemetry = parse_uart_telemetry_frame(bytes(frame))

    assert telemetry["uart8_imu_valid"] is True
    assert telemetry["gyro_x_dps"] == 1.25
    assert telemetry["gyro_y_dps"] == -2.5
    assert math.isclose(telemetry["accel_z_mps2"], 981 * 9.80665 / 1000.0)


def test_uart_telemetry_frame_three_marks_missing_uart8_sample_invalid():
    # An all-zero payload with a cleared valid flag is not a stationary
    # acceleration sample.  The bridge must drop it rather than merge it into
    # the previous legacy IMU sample.
    values = [0, 0, 0, 0, 0, 0, 0, 0]
    frame = bytearray([0xFF, 0xF8, 3])
    for value in values:
        frame.extend(int(value).to_bytes(2, "little", signed=True))
    frame.append(sum(frame) & 0xFF)

    telemetry = parse_uart_telemetry_frame(bytes(frame))

    assert telemetry["uart8_imu_valid"] is False


def test_normalized_to_direct_pwm_offsets_pads_to_sixteen():
    offsets = normalized_to_direct_pwm_offsets([1.0, -0.5], span_us=200)
    assert offsets[:3] == [200, -100, 0]
    assert len(offsets) == 16


def test_normalized_to_direct_pwm_offsets_can_start_at_channel_eight():
    offsets = normalized_to_direct_pwm_offsets([1.0, -0.5], span_us=200, channel_offset=8)
    assert offsets[:8] == [0] * 8
    assert offsets[8:11] == [200, -100, 0]
    assert len(offsets) == 16

