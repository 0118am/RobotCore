#!/usr/bin/env python3
"""Offline export of one RobotCore run without replaying control topics."""

from __future__ import annotations

import argparse
import bisect
import csv
import json
import os
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.utilities import get_message


BOARD_TOPIC = "/hardware/board_status"
TARGET_TOPIC = "/runtime/trajectory_target"
TRACKING_TOPIC = "/runtime/tracking_status"
THRUSTER_TOPIC = "/control/thruster_cmd"
CONTROL_TOPICS = {BOARD_TOPIC, TARGET_TOPIC, TRACKING_TOPIC, THRUSTER_TOPIC}


def message_stamp_ns(message: Any) -> int | None:
    header = getattr(message, "header", None)
    stamp = getattr(header, "stamp", None)
    if stamp is None:
        return None
    value = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
    return value if value > 0 else None


def iso_utc(timestamp_ns: int) -> str:
    seconds, nanoseconds = divmod(int(timestamp_ns), 1_000_000_000)
    base = datetime.fromtimestamp(seconds, tz=timezone.utc)
    return f"{base.strftime('%Y-%m-%dT%H:%M:%S')}.{nanoseconds:09d}Z"


def compact_json(value: Any) -> str:
    return json.dumps(value, ensure_ascii=False, separators=(",", ":"))


def atomic_csv(path: Path, fieldnames: list[str], rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, path)


def read_run(run_directory: Path):
    bag_directory = run_directory / "rosbag2" / "tracking"
    event_log = run_directory / "event_log.jsonl"
    if not (bag_directory / "metadata.yaml").is_file():
        raise FileNotFoundError(f"rosbag metadata not found below {bag_directory}")
    if not event_log.is_file():
        raise FileNotFoundError(f"event log not found: {event_log}")

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_directory), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    message_classes = {name: get_message(type_name) for name, type_name in topic_types.items()}
    all_rows: list[dict[str, Any]] = []
    control_messages: dict[str, list[tuple[int, int, Any]]] = {
        topic: [] for topic in CONTROL_TOPICS
    }

    while reader.has_next():
        topic, raw, bag_time_ns = reader.read_next()
        message = deserialize_message(raw, message_classes[topic])
        header_time_ns = message_stamp_ns(message)
        timestamp_ns = header_time_ns or int(bag_time_ns)
        all_rows.append(
            {
                "timestamp_ns": timestamp_ns,
                "timestamp_utc": iso_utc(timestamp_ns),
                "source": "rosbag2",
                "topic_or_event": topic,
                "message_type": topic_types[topic],
                "header_time_ns": header_time_ns or "",
                "record_time_ns": int(bag_time_ns),
                "record_delay_ms": (
                    (int(bag_time_ns) - header_time_ns) / 1e6
                    if header_time_ns is not None else ""
                ),
                "payload_json": compact_json(message_to_ordereddict(message)),
            }
        )
        if topic in control_messages and header_time_ns is not None:
            control_messages[topic].append((header_time_ns, int(bag_time_ns), message))

    with event_log.open(encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, start=1):
            if not line.strip():
                continue
            event = json.loads(line)
            timestamp_ns = int(event["time"])
            event_type = str(event.get("type", "unknown"))
            all_rows.append(
                {
                    "timestamp_ns": timestamp_ns,
                    "timestamp_utc": iso_utc(timestamp_ns),
                    "source": "event_log",
                    "topic_or_event": event_type,
                    "message_type": "RobotCoreEvent",
                    "header_time_ns": "",
                    "record_time_ns": "",
                    "record_delay_ms": "",
                    "payload_json": compact_json(event.get("payload", {})),
                    "event_line": line_number,
                }
            )

    all_rows.sort(
        key=lambda row: (
            int(row["timestamp_ns"]), str(row["source"]), str(row["topic_or_event"])
        )
    )
    for values in control_messages.values():
        values.sort(key=lambda item: item[0])
    return all_rows, control_messages


def causal_sample(
    stream: list[tuple[int, int, Any]], stamps: list[int], timestamp_ns: int
) -> tuple[int, int, Any] | None:
    if not stream:
        return None
    index = bisect.bisect_right(stamps, timestamp_ns) - 1
    return stream[index] if index >= 0 else None


def xyz(value: Any) -> tuple[float, float, float]:
    return float(value.x), float(value.y), float(value.z)


def quaternion_wxyz(value: Any) -> tuple[float, float, float, float]:
    return float(value.w), float(value.x), float(value.y), float(value.z)


def build_pwm_rows(
    control_messages: dict[str, list[tuple[int, int, Any]]]
) -> list[dict[str, Any]]:
    output: list[dict[str, Any]] = []
    stream_stamps = {
        topic: [item[0] for item in values]
        for topic, values in control_messages.items()
    }
    for board_stamp, board_record_stamp, board in control_messages[BOARD_TOPIC]:
        target_item = causal_sample(
            control_messages[TARGET_TOPIC], stream_stamps[TARGET_TOPIC], board_stamp
        )
        tracking_item = causal_sample(
            control_messages[TRACKING_TOPIC], stream_stamps[TRACKING_TOPIC], board_stamp
        )
        thruster_item = causal_sample(
            control_messages[THRUSTER_TOPIC], stream_stamps[THRUSTER_TOPIC], board_stamp
        )
        row: dict[str, Any] = {
            "pwm_stamp_ns": board_stamp,
            "pwm_stamp_utc": iso_utc(board_stamp),
            "pwm_record_time_ns": board_record_stamp,
            "pwm_record_delay_ms": (board_record_stamp - board_stamp) / 1e6,
            "board_tick_ms": int(board.board_tick_ms),
            "connected": bool(board.connected),
            "heartbeat_ok": bool(board.heartbeat_ok),
            "failsafe_active": bool(board.failsafe_active),
            "session_established": bool(board.session_established),
            "outputs_enabled": bool(board.outputs_enabled),
            "received_sequence": int(board.received_sequence),
            "applied_sequence": int(board.applied_sequence),
            "board_command_age_ms": int(board.command_age_ms),
            "rx_crc_errors": int(board.rx_crc_errors),
            "safety_reason": int(board.safety_reason),
        }
        for index, value in enumerate(board.pwm_us, start=8):
            row[f"pwm_{index}_us"] = int(value)

        if target_item is not None:
            target_stamp, _, target = target_item
            position = xyz(target.target_pose.position)
            orientation = quaternion_wxyz(target.target_pose.orientation)
            target_velocity = xyz(target.target_twist.linear)
            target_acceleration = xyz(target.target_accel.linear)
            row.update(
                {
                    "target_stamp_ns": target_stamp,
                    "target_age_ms": (board_stamp - target_stamp) / 1e6,
                    "target_x_m": position[0],
                    "target_y_m": position[1],
                    "target_z_m": position[2],
                    "target_qw": orientation[0],
                    "target_qx": orientation[1],
                    "target_qy": orientation[2],
                    "target_qz": orientation[3],
                    "target_vx_mps": target_velocity[0],
                    "target_vy_mps": target_velocity[1],
                    "target_vz_mps": target_velocity[2],
                    "target_ax_mps2": target_acceleration[0],
                    "target_ay_mps2": target_acceleration[1],
                    "target_az_mps2": target_acceleration[2],
                    "trajectory_type": target.trajectory_type,
                    "control_mode": target.control_mode,
                    "trajectory_phase": target.trajectory_phase,
                    "trajectory_time_s": float(target.time_s),
                    "target_valid": bool(target.valid),
                }
            )

        if tracking_item is not None:
            tracking_stamp, _, tracking = tracking_item
            actual_position = xyz(tracking.actual_position)
            actual_velocity = xyz(tracking.actual_velocity_body)
            position_error = xyz(tracking.position_error_body)
            row.update(
                {
                    "tracking_stamp_ns": tracking_stamp,
                    "tracking_age_ms": (board_stamp - tracking_stamp) / 1e6,
                    "actual_x_m": actual_position[0],
                    "actual_y_m": actual_position[1],
                    "actual_z_m": actual_position[2],
                    "actual_vx_body_mps": actual_velocity[0],
                    "actual_vy_body_mps": actual_velocity[1],
                    "actual_vz_body_mps": actual_velocity[2],
                    "position_error_x_body_m": position_error[0],
                    "position_error_y_body_m": position_error[1],
                    "position_error_z_body_m": position_error[2],
                    "position_error_m": float(tracking.position_error_m),
                    "orientation_error_rad": float(tracking.orientation_error_rad),
                    "velocity_error_mps": float(tracking.velocity_error_mps),
                    "tracking_valid": bool(tracking.valid),
                }
            )

        if thruster_item is not None:
            thruster_stamp, _, thruster = thruster_item
            row.update(
                {
                    "thruster_command_stamp_ns": thruster_stamp,
                    "thruster_command_age_ms": (board_stamp - thruster_stamp) / 1e6,
                    "thruster_enable": bool(thruster.enable),
                    "thruster_armed": bool(thruster.armed),
                    "thruster_arm_generation": int(thruster.arm_generation),
                    "thruster_source": thruster.source,
                }
            )
            for index, value in enumerate(thruster.action):
                row[f"thruster_{index}_action"] = float(value)
        output.append(row)
    return output


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Merge one RobotCore run offline; never replays ROS topics."
    )
    parser.add_argument("run", type=Path, help="run_<timestamp>_<task> directory")
    parser.add_argument(
        "--all-output",
        type=Path,
        help="lossless long-form CSV (default: LOG_ROOT/exports/RUN/all_records_merged.csv)",
    )
    parser.add_argument(
        "--control-output",
        type=Path,
        help="causally aligned PWM/control CSV (default: LOG_ROOT/exports/RUN/pwm_target_merged.csv)",
    )
    args = parser.parse_args()
    run_directory = args.run.resolve()
    export_directory = run_directory.parent.parent / "exports" / run_directory.name
    all_output = args.all_output or export_directory / "all_records_merged.csv"
    control_output = args.control_output or export_directory / "pwm_target_merged.csv"

    all_rows, control_messages = read_run(run_directory)
    all_fields = [
        "timestamp_ns",
        "timestamp_utc",
        "source",
        "topic_or_event",
        "message_type",
        "header_time_ns",
        "record_time_ns",
        "record_delay_ms",
        "event_line",
        "payload_json",
    ]
    atomic_csv(Path(all_output), all_fields, all_rows)

    control_rows = build_pwm_rows(control_messages)
    if not control_rows:
        raise RuntimeError(f"no {BOARD_TOPIC} messages found")
    control_fields = list(control_rows[0])
    for row in control_rows[1:]:
        for name in row:
            if name not in control_fields:
                control_fields.append(name)
    atomic_csv(Path(control_output), control_fields, control_rows)
    print(f"all records: {len(all_rows)} -> {all_output}")
    print(f"PWM-aligned records: {len(control_rows)} -> {control_output}")


if __name__ == "__main__":
    main()
