"""Aboard UART6 packet helpers used by the real ROS-side hardware bridge."""

import struct

A_BOARD_DIRECT_PWM_HEADER = bytes([0xFF, 0xFA])
A_BOARD_PWM_FEEDBACK_HEADER = bytes([0xFF, 0xFB])
A_BOARD_TELEMETRY_HEADER = bytes([0xFF, 0xF8])
A_BOARD_DIRECT_PWM_CHANNELS = 16
A_BOARD_PWM_FEEDBACK_LEN = 2 + A_BOARD_DIRECT_PWM_CHANNELS * 2 + 1
A_BOARD_TELEMETRY_LEN = 20
A_BOARD_ACCEL_LSB_PER_G = 4096.0
STANDARD_GRAVITY_MPS2 = 9.80665


def build_uart_direct_pwm_frame(offsets):
    """Build the STM32 UART6 direct-PWM command frame.

    Offsets are signed microseconds relative to 1500us and are clamped by the
    A-board firmware to its configured safe range.
    """
    padded = list(offsets)[:A_BOARD_DIRECT_PWM_CHANNELS]
    padded += [0] * (A_BOARD_DIRECT_PWM_CHANNELS - len(padded))
    frame = bytearray(A_BOARD_DIRECT_PWM_HEADER)
    frame.extend(struct.pack("<16h", *[int(value) for value in padded]))
    frame.append(sum(frame) & 0xFF)
    return bytes(frame)


def normalized_to_direct_pwm_offsets(values, span_us=100, channel_offset=0):
    """Map the fixed 8-channel normalized command to a 16-channel PWM frame."""
    channel_offset = max(0, min(8, int(channel_offset)))
    offsets = [0] * A_BOARD_DIRECT_PWM_CHANNELS
    for value in list(values)[:8]:
        clamped = max(-1.0, min(1.0, float(value)))
        offsets[channel_offset] = int(clamped * span_us)
        channel_offset += 1
    return offsets


def parse_uart_pwm_feedback_frame(frame: bytes):
    """Parse one FF FB feedback frame into 16 PWM microsecond readings."""
    if len(frame) != A_BOARD_PWM_FEEDBACK_LEN:
        raise ValueError(f"expected {A_BOARD_PWM_FEEDBACK_LEN} bytes, got {len(frame)}")
    if frame[:2] != A_BOARD_PWM_FEEDBACK_HEADER:
        raise ValueError("invalid A-board PWM feedback header")
    checksum = sum(frame[:-1]) & 0xFF
    if frame[-1] != checksum:
        raise ValueError("invalid A-board PWM feedback checksum")
    return list(struct.unpack("<16H", frame[2:-1]))


def parse_uart_telemetry_frame(frame: bytes):
    """Parse one FF F8 A-board telemetry frame.

    Frame 0 carries attitude and gyro readings. Frame 1 carries controller
    internals. Frame 2 carries raw MPU6500 accelerometer specific-force
    readings.  All supported IMU sources deliberately retain gravity here;
    the ROS calibration stage removes it only after attitude and bias have
    been validated.
    """
    if len(frame) != A_BOARD_TELEMETRY_LEN:
        raise ValueError(f"expected {A_BOARD_TELEMETRY_LEN} bytes, got {len(frame)}")
    if frame[:2] != A_BOARD_TELEMETRY_HEADER:
        raise ValueError("invalid A-board telemetry header")
    checksum = sum(frame[:-1]) & 0xFF
    if frame[-1] != checksum:
        raise ValueError("invalid A-board telemetry checksum")

    frame_num = frame[2]
    values = struct.unpack("<8h", frame[3:-1])
    if frame_num == 0:
        return {
            "pitch_deg": float(values[0]),
            "roll_deg": float(values[1]),
            "yaw_deg": float(values[2]),
            "depth_raw": int(values[3]),
            "depth_m": float(values[3]) / 1000.0,
            "depth_velocity": float(values[4]) / 100.0,
            "gyro_y_dps": float(values[5]) / 100.0,
            "gyro_x_dps": float(values[6]) / 100.0,
            "gyro_z_dps": float(values[7]) / 100.0,
        }
    if frame_num == 2:
        scale = STANDARD_GRAVITY_MPS2 / A_BOARD_ACCEL_LSB_PER_G
        return {
            "accel_x_mps2": float(values[0]) * scale,
            "accel_y_mps2": float(values[1]) * scale,
            "accel_z_mps2": float(values[2]) * scale,
        }
    if frame_num == 3:
        # A-board UART8 IMU. Angular velocity is
        # centi-degrees/s; acceleration is milli-g.  values[6] is a validity
        # flag, so stale UART8 values are not presented as fresh sensor data.
        return {
            "uart8_imu_valid": bool(values[6]),
            "gyro_x_dps": values[0] / 100.0,
            "gyro_y_dps": values[1] / 100.0,
            "gyro_z_dps": values[2] / 100.0,
            "accel_x_mps2": values[3] * STANDARD_GRAVITY_MPS2 / 1000.0,
            "accel_y_mps2": values[4] * STANDARD_GRAVITY_MPS2 / 1000.0,
            "accel_z_mps2": values[5] * STANDARD_GRAVITY_MPS2 / 1000.0,
        }
    return None
