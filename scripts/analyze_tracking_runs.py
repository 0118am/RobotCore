#!/usr/bin/env python3
"""Create reproducible PID/RL tracking metrics and plots from RobotCore runs."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

POSE_AXIS_NAMES = ("x", "y", "z", "roll", "pitch", "yaw")


def percentile(values, quantile):
    return float(np.percentile(values, quantile)) if len(values) else math.nan


def error_metrics(values, prefix):
    array = np.asarray(values, dtype=np.float64)
    if not len(array):
        return {
            f"{prefix}_rmse": math.nan,
            f"{prefix}_mae": math.nan,
            f"{prefix}_p95": math.nan,
            f"{prefix}_max": math.nan,
        }
    return {
        f"{prefix}_rmse": float(np.sqrt(np.mean(array**2))),
        f"{prefix}_mae": float(np.mean(np.abs(array))),
        f"{prefix}_p95": percentile(np.abs(array), 95),
        f"{prefix}_max": float(np.max(np.abs(array))),
    }


def finite_array(values, columns=None):
    """Return a predictable two-dimensional array from partially populated logs."""

    array = np.asarray(values, dtype=np.float64)
    if not array.size:
        return np.empty((0, int(columns or 0)), dtype=np.float64)
    if array.ndim == 1:
        array = array.reshape(-1, 1)
    if columns is not None and array.shape[1] != columns:
        return np.full((len(array), columns), math.nan, dtype=np.float64)
    return array


def quaternion_wxyz_to_rpy(quaternions):
    """Convert normalized wxyz quaternions to continuous roll/pitch/yaw radians."""

    values = finite_array(quaternions, 4)
    result = np.full((len(values), 3), math.nan, dtype=np.float64)
    for index, quaternion in enumerate(values):
        if not np.all(np.isfinite(quaternion)):
            continue
        norm = np.linalg.norm(quaternion)
        if norm <= 1e-12:
            continue
        w, x, y, z = quaternion / norm
        result[index, 0] = math.atan2(
            2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y)
        )
        result[index, 1] = math.asin(
            float(np.clip(2.0 * (w * y - z * x), -1.0, 1.0))
        )
        result[index, 2] = math.atan2(
            2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)
        )
    return np.unwrap(result, axis=0)


def tracking_pose_arrays(tracking):
    target_position = finite_array(
        [item["payload"].get("target_position", [math.nan] * 3) for item in tracking],
        3,
    )
    actual_position = finite_array(
        [item["payload"].get("actual_position", [math.nan] * 3) for item in tracking],
        3,
    )
    target_rpy = quaternion_wxyz_to_rpy(
        [
            item["payload"].get("target_orientation_wxyz", [math.nan] * 4)
            for item in tracking
        ]
    )
    actual_rpy = quaternion_wxyz_to_rpy(
        [
            item["payload"].get("actual_orientation_wxyz", [math.nan] * 4)
            for item in tracking
        ]
    )
    return (
        np.column_stack((target_position, target_rpy)),
        np.column_stack((actual_position, actual_rpy)),
    )


def estimate_trajectory_delay(time_s, target_pose, actual_pose):
    """Estimate non-negative lag from the most energetic pose axis."""

    if len(time_s) < 10:
        return math.nan
    centered_target = target_pose - np.nanmean(target_pose, axis=0)
    axis_energy = np.nanstd(centered_target, axis=0)
    if not np.any(np.isfinite(axis_energy)) or np.nanmax(axis_energy) <= 1e-6:
        return math.nan
    axis = int(np.nanargmax(axis_energy))
    target = centered_target[:, axis]
    actual = actual_pose[:, axis] - np.nanmean(actual_pose[:, axis])
    valid = np.isfinite(target) & np.isfinite(actual)
    if np.count_nonzero(valid) < 10:
        return math.nan
    target = target[valid]
    actual = actual[valid]
    correlation = np.correlate(actual, target, mode="full")
    lags = np.arange(-len(target) + 1, len(actual))
    nonnegative = lags >= 0
    lag_samples = int(lags[nonnegative][np.argmax(correlation[nonnegative])])
    dt = np.diff(time_s[np.isfinite(time_s)])
    dt = dt[(dt > 0.0) & (dt < 1.0)]
    return float(lag_samples * np.median(dt)) if len(dt) else math.nan


def estimate_step_settling_time(time_s, target_pose, actual_pose, trajectory_type):
    """Return time from detected step to permanent entry in a 2%/noise floor band."""

    if not trajectory_type.startswith("step_") or len(time_s) < 3:
        return math.nan
    axis_name = trajectory_type.removeprefix("step_")
    if axis_name not in POSE_AXIS_NAMES:
        return math.nan
    axis = POSE_AXIS_NAMES.index(axis_name)
    target = target_pose[:, axis]
    actual = actual_pose[:, axis]
    changes = np.abs(np.diff(target))
    if not np.any(np.isfinite(changes)) or np.nanmax(changes) <= 1e-6:
        return math.nan
    step_index = int(np.nanargmax(changes)) + 1
    amplitude = abs(float(target[step_index] - target[0]))
    floor = math.radians(2.0) if axis >= 3 else 0.02
    tolerance = max(0.02 * amplitude, floor)
    within = np.abs(actual - target) <= tolerance
    for index in range(step_index, len(within)):
        if np.all(within[index:]):
            return float(time_s[index] - time_s[step_index])
    return math.nan


def load_events(run_dir):
    events = []
    with (run_dir / "event_log.jsonl").open("r", encoding="utf-8") as handle:
        for line in handle:
            try:
                events.append(json.loads(line))
            except json.JSONDecodeError:
                continue
    return events


def select_experiment_window(events):
    """Return the latest explicitly delimited tracking experiment window."""

    markers = [
        event for event in events if event.get("type") == "tracking_experiment"
    ]
    starts = [
        event for event in markers if event.get("payload", {}).get("phase") == "start"
    ]
    if not starts:
        raise ValueError("run is missing a tracking_experiment start marker")
    start = starts[-1]
    start_time = int(start.get("time", 0))
    ends = [
        event
        for event in markers
        if event.get("payload", {}).get("phase") == "end"
        and int(event.get("time", 0)) >= start_time
    ]
    end_time = int(ends[0].get("time", 2**63 - 1)) if ends else 2**63 - 1
    metadata = dict(start.get("payload", {}))
    if ends:
        metadata["completed_successfully"] = bool(
            ends[0].get("payload", {}).get("success")
        )
    return [
        event
        for event in events
        if start_time <= int(event.get("time", 0)) <= end_time
    ], metadata


def analyze_run(run_dir, output_dir):
    events, experiment = select_experiment_window(load_events(run_dir))
    tracking = [event for event in events if event.get("type") == "tracking_status"]
    commands = [
        event
        for event in events
        if event.get("type") == "thruster_cmd"
        and event.get("payload", {}).get("enable")
    ]
    pid_status = [event for event in events if event.get("type") == "pid_status"]
    authority = [
        event for event in events if event.get("type") == "control_authority_status"
    ]
    safety = [
        event
        for event in events
        if event.get("type") == "safety_event"
        and event.get("payload", {}).get("abort_active")
    ]
    authority_faults = [
        event
        for event in authority
        if event.get("payload", {}).get("fault_latched")
        or event.get("payload", {}).get("abort_active")
    ]

    valid = [bool(item["payload"].get("valid")) for item in tracking]
    position_error = [
        float(item["payload"].get("position_error_m", math.nan))
        for item in tracking
        if item["payload"].get("valid")
    ]
    orientation_error_deg = [
        math.degrees(float(item["payload"].get("orientation_error_rad", math.nan)))
        for item in tracking
        if item["payload"].get("valid")
    ]
    velocity_error = [
        float(item["payload"].get("velocity_error_mps", math.nan))
        for item in tracking
        if item["payload"].get("valid")
    ]
    angular_velocity_error = [
        float(item["payload"].get("angular_velocity_error_rps", math.nan))
        for item in tracking
        if item["payload"].get("valid")
    ]
    metrics = {
        "run": run_dir.name,
        "scenario": str(experiment.get("scenario", "")),
        "controller": str(experiment.get("controller", "")),
        "tracking_samples": len(tracking),
        "valid_fraction": float(np.mean(valid)) if valid else 0.0,
        "abort_count": len(safety),
        "authority_fault_samples": len(authority_faults),
    }
    metrics.update(error_metrics(position_error, "position_m"))
    metrics.update(error_metrics(orientation_error_deg, "orientation_deg"))
    metrics.update(error_metrics(velocity_error, "velocity_mps"))
    metrics.update(error_metrics(angular_velocity_error, "angular_velocity_rps"))
    time_s = np.asarray(
        [
            float(item["payload"].get("time_s", index))
            for index, item in enumerate(tracking)
        ],
        dtype=np.float64,
    )
    target_pose, actual_pose = tracking_pose_arrays(tracking)
    trajectory_type = (
        str(tracking[-1]["payload"].get("trajectory_type", "")) if tracking else ""
    )
    metrics["trajectory_delay_s"] = estimate_trajectory_delay(
        time_s, target_pose, actual_pose
    )
    metrics["step_settling_time_s"] = estimate_step_settling_time(
        time_s, target_pose, actual_pose, trajectory_type
    )

    command_values = np.asarray(
        [item["payload"].get("action", [0.0] * 8) for item in commands],
        dtype=np.float64,
    )
    command_times = np.asarray([item.get("time", 0) for item in commands], dtype=np.float64) * 1e-9
    command_limit = 1.0
    if authority:
        command_limit = float(authority[-1]["payload"].get("action_limit", command_limit))
    if command_values.size:
        metrics["command_rms"] = float(np.sqrt(np.mean(command_values**2)))
        metrics["command_total_variation"] = float(
            np.sum(np.abs(np.diff(command_values, axis=0)))
        )
        metrics["saturation_fraction"] = float(
            np.mean(np.any(np.abs(command_values) >= command_limit - 1e-4, axis=1))
        )
        if len(command_times) > 1:
            dt = np.maximum(0.0, np.diff(command_times))
            power_proxy = np.sum(command_values[:-1] ** 2, axis=1)
            metrics["command_squared_integral"] = float(np.sum(power_proxy * dt))
        else:
            metrics["command_squared_integral"] = 0.0
    else:
        metrics.update(
            {
                "command_rms": math.nan,
                "command_total_variation": math.nan,
                "saturation_fraction": math.nan,
                "command_squared_integral": math.nan,
            }
        )
    residuals = [
        float(item["payload"].get("allocation_residual", math.nan))
        for item in pid_status
        if math.isfinite(float(item["payload"].get("allocation_residual", math.nan)))
    ]
    metrics["allocation_residual_mean"] = (
        float(np.mean(residuals)) if residuals else math.nan
    )
    metrics["pass"] = bool(
        metrics["tracking_samples"] > 0
        and metrics["valid_fraction"] >= 0.99
        and metrics["position_m_rmse"] <= 0.15
        and metrics["orientation_deg_rmse"] <= 10.0
        and metrics["saturation_fraction"] <= 0.05
        and metrics["abort_count"] == 0
        and metrics["authority_fault_samples"] == 0
    )
    plot_run(run_dir.name, tracking, command_values, output_dir)
    return metrics


def plot_run(name, tracking, command_values, output_dir):
    if not tracking:
        return
    time_s = np.asarray(
        [float(item["payload"].get("time_s", index)) for index, item in enumerate(tracking)]
    )
    target_pose, actual_pose = tracking_pose_arrays(tracking)
    target = target_pose[:, :3]
    actual = actual_pose[:, :3]
    position_error = np.asarray(
        [item["payload"].get("position_error_m", math.nan) for item in tracking]
    )
    orientation_error = np.degrees(
        np.asarray(
            [item["payload"].get("orientation_error_rad", math.nan) for item in tracking]
        )
    )
    figure = plt.figure(figsize=(14, 10))
    axis_3d = figure.add_subplot(221, projection="3d")
    axis_3d.plot(*target.T, label="target")
    axis_3d.plot(*actual.T, label="actual")
    axis_3d.set_title("3D trajectory")
    axis_3d.legend()
    axis_position = figure.add_subplot(222)
    axis_position.plot(time_s, position_error)
    axis_position.axhline(0.15, color="red", linestyle="--", label="RMSE target")
    axis_position.set_title("Position error (m)")
    axis_position.legend()
    axis_orientation = figure.add_subplot(223)
    axis_orientation.plot(time_s, orientation_error)
    axis_orientation.axhline(10.0, color="red", linestyle="--", label="RMSE target")
    axis_orientation.set_title("Orientation geodesic error (deg)")
    axis_orientation.legend()
    axis_command = figure.add_subplot(224)
    if command_values.size:
        axis_command.plot(command_values)
    axis_command.set_title("Final direct thruster actions")
    figure.suptitle(name)
    figure.tight_layout()
    figure.savefig(output_dir / f"{name}_tracking.png", dpi=150)
    plt.close(figure)

    pose_figure, axes = plt.subplots(3, 2, figsize=(14, 11), sharex=True)
    for index, axis in enumerate(axes.flat):
        scale = 180.0 / math.pi if index >= 3 else 1.0
        unit = "deg" if index >= 3 else "m"
        axis.plot(time_s, target_pose[:, index] * scale, label="target")
        axis.plot(time_s, actual_pose[:, index] * scale, label="actual")
        axis.set_ylabel(f"{POSE_AXIS_NAMES[index]} ({unit})")
        axis.grid(True, alpha=0.3)
    axes[0, 0].legend()
    axes[-1, 0].set_xlabel("trajectory time (s)")
    axes[-1, 1].set_xlabel("trajectory time (s)")
    pose_figure.suptitle(f"{name}: six-axis target and measured pose")
    pose_figure.tight_layout()
    pose_figure.savefig(output_dir / f"{name}_six_axis_pose.png", dpi=150)
    plt.close(pose_figure)

    position_error_body = finite_array(
        [
            item["payload"].get(
                "position_error_body",
                target_pose[index, :3] - actual_pose[index, :3],
            )
            for index, item in enumerate(tracking)
        ],
        3,
    )
    orientation_error_body = finite_array(
        [
            item["payload"].get(
                "orientation_error_body",
                target_pose[index, 3:] - actual_pose[index, 3:],
            )
            for index, item in enumerate(tracking)
        ],
        3,
    )
    error_axes = np.column_stack((position_error_body, orientation_error_body))
    error_figure, axes = plt.subplots(3, 2, figsize=(14, 11), sharex=True)
    for index, axis in enumerate(axes.flat):
        scale = 180.0 / math.pi if index >= 3 else 1.0
        unit = "deg" if index >= 3 else "m"
        axis.plot(time_s, error_axes[:, index] * scale)
        axis.axhline(0.0, color="black", linewidth=0.7)
        axis.set_ylabel(f"{POSE_AXIS_NAMES[index]} error ({unit})")
        axis.grid(True, alpha=0.3)
    axes[-1, 0].set_xlabel("trajectory time (s)")
    axes[-1, 1].set_xlabel("trajectory time (s)")
    error_figure.suptitle(f"{name}: body-frame tracking errors")
    error_figure.tight_layout()
    error_figure.savefig(output_dir / f"{name}_six_axis_error.png", dpi=150)
    plt.close(error_figure)

    velocity_target = np.column_stack(
        (
            finite_array(
                [
                    item["payload"].get("target_velocity_body", [math.nan] * 3)
                    for item in tracking
                ],
                3,
            ),
            finite_array(
                [
                    item["payload"].get(
                        "target_angular_velocity_body", [math.nan] * 3
                    )
                    for item in tracking
                ],
                3,
            ),
        )
    )
    velocity_actual = np.column_stack(
        (
            finite_array(
                [
                    item["payload"].get("actual_velocity_body", [math.nan] * 3)
                    for item in tracking
                ],
                3,
            ),
            finite_array(
                [
                    item["payload"].get(
                        "actual_angular_velocity_body", [math.nan] * 3
                    )
                    for item in tracking
                ],
                3,
            ),
        )
    )
    velocity_figure, axes = plt.subplots(3, 2, figsize=(14, 11), sharex=True)
    for index, axis in enumerate(axes.flat):
        axis.plot(time_s, velocity_target[:, index], label="target")
        axis.plot(time_s, velocity_actual[:, index], label="actual")
        unit = "rad/s" if index >= 3 else "m/s"
        axis.set_ylabel(f"{POSE_AXIS_NAMES[index]} ({unit})")
        axis.grid(True, alpha=0.3)
    axes[0, 0].legend()
    axes[-1, 0].set_xlabel("trajectory time (s)")
    axes[-1, 1].set_xlabel("trajectory time (s)")
    velocity_figure.suptitle(f"{name}: six-axis body velocity")
    velocity_figure.tight_layout()
    velocity_figure.savefig(output_dir / f"{name}_six_axis_velocity.png", dpi=150)
    plt.close(velocity_figure)

    if command_values.size:
        command_figure, axis = plt.subplots(figsize=(14, 6))
        axis.plot(command_values)
        axis.set_xlabel("command sample")
        axis.set_ylabel("action")
        axis.set_ylim(-1.02, 1.02)
        axis.grid(True, alpha=0.3)
        axis.legend([f"T{index + 1}" for index in range(command_values.shape[1])], ncol=4)
        axis.set_title(f"{name}: final thruster commands")
        command_figure.tight_layout()
        command_figure.savefig(output_dir / f"{name}_thrusters.png", dpi=150)
        plt.close(command_figure)


def finite_json(value):
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {key: finite_json(item) for key, item in value.items()}
    if isinstance(value, list):
        return [finite_json(item) for item in value]
    return value


def plot_summary(metrics, output_dir):
    if not metrics:
        return
    names = [row["run"] for row in metrics]
    figure, axes = plt.subplots(2, 2, figsize=(14, 9))
    plots = (
        ("position_m_rmse", "Position RMSE (m)", 0.15),
        ("orientation_deg_rmse", "Orientation RMSE (deg)", 10.0),
        ("saturation_fraction", "Saturated sample fraction", 0.05),
        ("valid_fraction", "Valid sample fraction", 0.99),
    )
    for axis, (key, title, threshold) in zip(axes.flat, plots):
        values = [
            float(row.get(key, math.nan))
            if row.get(key) is not None
            else math.nan
            for row in metrics
        ]
        colors = ["tab:green" if row.get("pass") else "tab:red" for row in metrics]
        axis.bar(names, values, color=colors)
        axis.axhline(threshold, color="black", linestyle="--", linewidth=1.0)
        axis.set_title(title)
        axis.tick_params(axis="x", rotation=25)
        axis.grid(True, axis="y", alpha=0.3)
    figure.suptitle("Tracking run comparison (green = all acceptance criteria pass)")
    figure.tight_layout()
    figure.savefig(output_dir / "run_comparison.png", dpi=150)
    plt.close(figure)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("runs", nargs="+", type=Path)
    parser.add_argument("--output", type=Path, default=Path("tracking_report"))
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    metrics = [analyze_run(path.resolve(), args.output) for path in args.runs]
    keys = sorted({key for row in metrics for key in row})
    with (args.output / "summary.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=keys)
        writer.writeheader()
        writer.writerows(metrics)
    (args.output / "summary.json").write_text(
        json.dumps(finite_json(metrics), indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    plot_summary(metrics, args.output)
    print(args.output / "summary.csv")


if __name__ == "__main__":
    main()
