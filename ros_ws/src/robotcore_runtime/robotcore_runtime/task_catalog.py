"""Strict, immutable loading for managed tracking and recording YAML."""

from __future__ import annotations

import copy
import hashlib
import math
import re
from pathlib import Path
from typing import Any

import yaml


_IDENTIFIER = re.compile(r"[a-z0-9][a-z0-9_-]{0,63}\Z")
_CONTROL_MODES = {"altitude_hold", "station_hold", "station_hold_fast", "rl_policy"}
_STRING_TRAJECTORY_FIELDS = {"trajectory_type", "attitude_mode"}
_BOOL_TRAJECTORY_FIELDS = {"relative_to_initial_pose"}
_NUMBER_TRAJECTORY_FIELDS = {
    "center_x",
    "center_y",
    "center_z",
    "hold_before_motion_s",
    "amp_x",
    "amp_y",
    "amp_z",
    "radius_m",
    "period_s",
    "trajectory_speed_mps",
    "trajectory_ramp_s",
    "move_duration_s",
    "manual_vertical_speed_mps",
    "station_linear_input_gain_mps",
    "station_lateral_input_gain_mps",
    "station_yaw_input_gain_rps",
}
_TRAJECTORY_FIELDS = (
    _STRING_TRAJECTORY_FIELDS | _BOOL_TRAJECTORY_FIELDS | _NUMBER_TRAJECTORY_FIELDS
)
_EVENT_LOG_STREAMS = {
    "thruster_cmd",
    "trajectory_target",
    "tracking_status",
    "control_authority_status",
    "pid_status",
}


class _UniqueKeyLoader(yaml.SafeLoader):
    """SafeLoader variant that rejects duplicate YAML mapping keys."""


def _construct_unique_mapping(loader, node, deep=False):
    mapping = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if key in mapping:
            raise ValueError(f"duplicate YAML key: {key}")
        mapping[key] = loader.construct_object(value_node, deep=deep)
    return mapping


_UniqueKeyLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG,
    _construct_unique_mapping,
)


def _load_mapping(path: Path) -> tuple[dict[str, Any], str]:
    raw = path.read_bytes()
    try:
        document = yaml.load(raw.decode("utf-8"), Loader=_UniqueKeyLoader)
    except (UnicodeDecodeError, yaml.YAMLError) as exc:
        raise ValueError(f"invalid YAML in {path}: {exc}") from exc
    if not isinstance(document, dict):
        raise ValueError(f"YAML document must be a mapping: {path}")
    return document, hashlib.sha256(raw).hexdigest()


def _exact_keys(value: dict[str, Any], allowed: set[str], label: str) -> None:
    unknown = set(value) - allowed
    if unknown:
        raise ValueError(f"{label} contains unknown keys: {', '.join(sorted(unknown))}")


def _identifier(value: Any, label: str) -> str:
    text = str(value)
    if _IDENTIFIER.fullmatch(text) is None:
        raise ValueError(f"{label} must match {_IDENTIFIER.pattern}")
    return text


def _finite_number(value: Any, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{label} must be a number")
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(f"{label} must be finite")
    return number


class TaskCatalog:
    """Validated task presets resolved from one runtime-owned YAML document."""

    def __init__(self, path: str | Path):
        self.path = Path(path)
        document, self.sha256 = _load_mapping(self.path)
        _exact_keys(
            document,
            {"schema_version", "defaults", "scenarios", "tasks"},
            "tracking task catalog",
        )
        if document.get("schema_version") != 1:
            raise ValueError("tracking task catalog schema_version must be 1")

        defaults = document.get("defaults")
        scenarios = document.get("scenarios")
        tasks = document.get("tasks")
        if not isinstance(defaults, dict):
            raise ValueError("tracking task defaults must be a mapping")
        if not isinstance(scenarios, dict) or not scenarios:
            raise ValueError("tracking task scenarios must be a non-empty mapping")
        if not isinstance(tasks, dict) or not tasks:
            raise ValueError("tracking tasks must be a non-empty mapping")
        _exact_keys(defaults, {"duration_s", "run_until_stopped"}, "task defaults")
        duration_default = _finite_number(defaults.get("duration_s"), "defaults.duration_s")
        if duration_default <= 0.0:
            raise ValueError("defaults.duration_s must be positive")
        run_default = defaults.get("run_until_stopped")
        if not isinstance(run_default, bool):
            raise ValueError("defaults.run_until_stopped must be a boolean")

        validated_scenarios: dict[str, dict[str, Any]] = {}
        for raw_name, raw_scenario in scenarios.items():
            name = _identifier(raw_name, "scenario id")
            if not isinstance(raw_scenario, dict):
                raise ValueError(f"scenario {name} must be a mapping")
            _exact_keys(raw_scenario, _TRAJECTORY_FIELDS, f"scenario {name}")
            if not raw_scenario:
                raise ValueError(f"scenario {name} must not be empty")
            scenario: dict[str, Any] = {}
            for key, value in raw_scenario.items():
                if key in _STRING_TRAJECTORY_FIELDS:
                    if not isinstance(value, str) or not value:
                        raise ValueError(f"scenario {name}.{key} must be a non-empty string")
                    scenario[key] = value
                elif key in _BOOL_TRAJECTORY_FIELDS:
                    if not isinstance(value, bool):
                        raise ValueError(f"scenario {name}.{key} must be a boolean")
                    scenario[key] = value
                else:
                    scenario[key] = _finite_number(value, f"scenario {name}.{key}")
            if "trajectory_type" not in scenario or "attitude_mode" not in scenario:
                raise ValueError(f"scenario {name} must declare trajectory_type and attitude_mode")
            validated_scenarios[name] = scenario

        resolved: dict[str, dict[str, Any]] = {}
        for raw_task_id, raw_task in tasks.items():
            task_id = _identifier(raw_task_id, "task id")
            if not isinstance(raw_task, dict):
                raise ValueError(f"task {task_id} must be a mapping")
            _exact_keys(
                raw_task,
                {
                    "label",
                    "scenario",
                    "control_mode",
                    "duration_s",
                    "run_until_stopped",
                },
                f"task {task_id}",
            )
            label = raw_task.get("label")
            scenario_id = raw_task.get("scenario")
            control_mode = raw_task.get("control_mode")
            if not isinstance(label, str) or not label.strip():
                raise ValueError(f"task {task_id}.label must be a non-empty string")
            if not isinstance(scenario_id, str) or scenario_id not in validated_scenarios:
                raise ValueError(f"task {task_id} references an unknown scenario")
            if control_mode not in _CONTROL_MODES:
                raise ValueError(f"task {task_id} has unsupported control_mode")
            controller = "rl" if control_mode == "rl_policy" else "pid"
            trajectory = validated_scenarios[scenario_id]
            trajectory_type = str(trajectory["trajectory_type"])
            if control_mode == "altitude_hold" and trajectory_type != "hold":
                raise ValueError(f"task {task_id} altitude_hold requires a hold scenario")
            duration = _finite_number(
                raw_task.get("duration_s", duration_default), f"task {task_id}.duration_s"
            )
            if duration <= 0.0:
                raise ValueError(f"task {task_id}.duration_s must be positive")
            run_until_stopped = raw_task.get("run_until_stopped", run_default)
            if not isinstance(run_until_stopped, bool):
                raise ValueError(f"task {task_id}.run_until_stopped must be a boolean")
            resolved[task_id] = {
                "schema_version": 1,
                "kind": "tracking_task",
                "name": task_id,
                "label": label.strip(),
                "scenario": scenario_id,
                "trajectory_name": "none" if trajectory_type == "hold" else scenario_id,
                "control_mode": control_mode,
                "controller": controller,
                "duration_s": duration,
                "run_until_stopped": run_until_stopped,
                "trajectory": copy.deepcopy(trajectory),
            }
        self._tasks = resolved

    def task(self, task_id: str) -> dict[str, Any]:
        task_id = _identifier(task_id, "task id")
        try:
            return copy.deepcopy(self._tasks[task_id])
        except KeyError as exc:
            raise KeyError(f"unknown managed tracking task: {task_id}") from exc

    def summaries(self) -> list[dict[str, str]]:
        return [
            {"id": task_id, "label": str(task["label"])}
            for task_id, task in self._tasks.items()
        ]


def load_recording_config(
    path: str | Path,
) -> tuple[list[str], dict[str, float], float, str]:
    config_path = Path(path)
    document, digest = _load_mapping(config_path)
    _exact_keys(
        document,
        {"schema_version", "event_log", "topics"},
        "recording configuration",
    )
    if document.get("schema_version") != 1:
        raise ValueError("recording schema_version must be 1")

    event_log = document.get("event_log")
    if not isinstance(event_log, dict):
        raise ValueError("recording event_log must be a mapping")
    _exact_keys(
        event_log,
        {"flush_interval_s", "stream_rates_hz"},
        "recording event_log",
    )
    flush_interval_s = _finite_number(
        event_log.get("flush_interval_s"), "recording event_log.flush_interval_s"
    )
    if not 0.05 <= flush_interval_s <= 10.0:
        raise ValueError("recording event_log.flush_interval_s must be within [0.05, 10]")
    raw_rates = event_log.get("stream_rates_hz")
    if not isinstance(raw_rates, dict):
        raise ValueError("recording event_log.stream_rates_hz must be a mapping")
    _exact_keys(raw_rates, _EVENT_LOG_STREAMS, "recording event_log.stream_rates_hz")
    missing_rates = _EVENT_LOG_STREAMS - set(raw_rates)
    if missing_rates:
        raise ValueError(
            "recording event_log.stream_rates_hz is missing: "
            + ", ".join(sorted(missing_rates))
        )
    stream_rates_hz: dict[str, float] = {}
    for stream in sorted(_EVENT_LOG_STREAMS):
        rate_hz = _finite_number(
            raw_rates[stream], f"recording event_log.stream_rates_hz.{stream}"
        )
        if not 0.0 < rate_hz <= 1000.0:
            raise ValueError(
                f"recording event_log.stream_rates_hz.{stream} must be within (0, 1000]"
            )
        stream_rates_hz[stream] = rate_hz

    raw_topics = document.get("topics")
    if not isinstance(raw_topics, list) or not raw_topics:
        raise ValueError("recording topics must be a non-empty list")
    topics: list[str] = []
    for value in raw_topics:
        if not isinstance(value, str) or not value.startswith("/") or value == "/":
            raise ValueError("every recording topic must be an absolute ROS topic name")
        if value in topics:
            raise ValueError(f"duplicate recording topic: {value}")
        topics.append(value)
    return topics, stream_rates_hz, flush_interval_s, digest
