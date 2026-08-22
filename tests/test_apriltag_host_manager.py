"""Host-owned AprilTag map tests without ROS 2 or hardware dependencies."""

import json
import sys
import tempfile
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "host_manager"))

from robotcore_host_manager.daemon import HostManager


def make_manager(root: Path) -> HostManager:
    (root / "apriltag_map.json").write_text(
        json.dumps(
            {
                "schema_version": 1,
                "frame": "map",
                "tags": {
                    "1": {
                        "size_m": 0.13,
                        "position_m": [0.0, 0.0, 0.0],
                        "rpy_deg": [0.0, 0.0, 0.0],
                    }
                },
            }
        ),
        encoding="utf-8",
    )
    return HostManager(
        {
            "schema_version": 1,
            "services": {"robot": "robotcore.service"},
            "apriltag_map": {
                "path": str(root / "apriltag_map.json"),
                "web_edit_enabled": True,
            },
        }
    )


def make_control_manager(root: Path) -> HostManager:
    pid_root = root / "pid"
    task_root = root / "tasks"
    pid_root.mkdir(parents=True)
    task_root.mkdir(parents=True)
    default_pid = json.loads(
        (ROOT / "ros_ws/src/robotcore_control/config/pid/default.json").read_text(
            encoding="utf-8"
        )
    )
    (pid_root / "active.json").write_text(
        json.dumps(default_pid), encoding="utf-8"
    )
    for source in (ROOT / "ros_ws/src/robotcore_runtime/config/tasks").glob("*.json"):
        (task_root / source.name).write_bytes(source.read_bytes())
    return HostManager(
        {
            "schema_version": 1,
            "services": {},
            "control_config": {
                "pid_active_path": str(pid_root / "active.json"),
                "pid_profiles_dir": str(pid_root / "profiles"),
                "tasks_dir": str(task_root),
            },
        }
    )


def test_apriltag_map_upsert_is_validated_and_atomic():
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        manager = make_manager(root)
        assert manager.handle({"action": "apriltag-map"})["tags"][0]["id"] == 1

        first = manager.handle(
            {
                "action": "apriltag-upsert",
                "tag": {
                    "id": 4,
                    "size_m": 0.13,
                    "position_m": [1.0, -2.0, 0.5],
                    "rpy_deg": [90.0, 0.0, -90.0],
                },
            }
        )
        assert first["accepted"] is True
        assert first["tag"]["id"] == 4

        manager.handle(
            {
                "action": "apriltag-upsert",
                "tag": {
                    "id": 4,
                    "size_m": 0.15,
                    "position_m": [1.1, -2.0, 0.5],
                    "rpy_deg": [90.0, 0.0, -90.0],
                },
            }
        )
        saved = json.loads((root / "apriltag_map.json").read_text(encoding="utf-8"))
        assert saved["tags"]["4"]["size_m"] == 0.15


def test_apriltag_map_rejects_invalid_tag_data_and_disabled_web_edits():
    with tempfile.TemporaryDirectory() as directory:
        manager = make_manager(Path(directory))
        invalid = {"action": "apriltag-upsert", "tag": {"id": 2, "size_m": 0, "position_m": [0, 0, 0], "rpy_deg": [0, 0, 0]}}
        try:
            manager.handle(invalid)
            raise AssertionError("zero-sized tag should be rejected")
        except ValueError as exc:
            assert "size_m" in str(exc)

        manager.config["apriltag_map"]["web_edit_enabled"] = False
        try:
            manager.handle({"action": "apriltag-upsert", "tag": {"id": 2, "size_m": 0.13, "position_m": [0, 0, 0], "rpy_deg": [0, 0, 0]}})
            raise AssertionError("disabled web edits should be rejected")
        except ValueError as exc:
            assert "disabled" in str(exc)

        try:
            manager.handle({"action": "apriltag-delete", "tag_id": 2})
            raise AssertionError("disabled map editing should reject deletion")
        except ValueError as exc:
            assert "disabled" in str(exc)


def test_apriltag_map_delete_is_validated_and_atomic():
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        manager = make_manager(root)
        manager.handle(
            {
                "action": "apriltag-upsert",
                "tag": {
                    "id": 9,
                    "size_m": 0.13,
                    "position_m": [1.0, 2.0, 3.0],
                    "rpy_deg": [0.0, 0.0, 0.0],
                },
            }
        )

        deleted = manager.handle({"action": "apriltag-delete", "tag_id": 9})

        assert deleted["accepted"] is True
        assert deleted["tag_id"] == 9
        assert [tag["id"] for tag in deleted["tags"]] == [1]
        saved = json.loads((root / "apriltag_map.json").read_text(encoding="utf-8"))
        assert list(saved["tags"]) == ["1"]

        try:
            manager.handle({"action": "apriltag-delete", "tag_id": 9})
            raise AssertionError("deleting a missing tag should be rejected")
        except ValueError as exc:
            assert "does not exist" in str(exc)


def test_apriltag_map_rename_replaces_the_original_id_atomically():
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        manager = make_manager(root)
        manager.handle(
            {
                "action": "apriltag-upsert",
                "tag": {"id": 9, "size_m": 0.13, "position_m": [1, 2, 3], "rpy_deg": [0, 0, 0]},
            }
        )

        renamed = manager.handle(
            {
                "action": "apriltag-upsert",
                "replace_tag_id": 9,
                "tag": {"id": 10, "size_m": 0.13, "position_m": [1, 2, 3], "rpy_deg": [0, 0, 0]},
            }
        )

        assert renamed["previous_tag_id"] == 9
        assert [tag["id"] for tag in renamed["tags"]] == [1, 10]
        saved = json.loads((root / "apriltag_map.json").read_text(encoding="utf-8"))
        assert set(saved["tags"]) == {"1", "10"}


def test_lifecycle_failure_includes_service_status_and_recent_logs(monkeypatch):
    manager = HostManager({"schema_version": 1, "services": {"robot": "robotcore.service"}})
    calls = []

    def fake_run(command, timeout_s=8.0):
        calls.append(command)
        if command[:2] == ["systemctl", "start"]:
            return {"ok": False, "returncode": 1, "output": "", "error": ""}
        if command[:2] == ["systemctl", "show"]:
            return {"ok": True, "returncode": 0, "output": "ActiveState=failed", "error": ""}
        return {"ok": True, "returncode": 0, "output": "unit failed: missing device", "error": ""}

    monkeypatch.setattr(manager, "_run", fake_run)
    result = manager.handle({"action": "start", "service": "robot"})

    assert result["ok"] is False
    assert result["service"] == "robotcore.service"
    assert result["status"]["output"] == "ActiveState=failed"
    assert result["recent_logs"]["output"] == "unit failed: missing device"
    assert ["systemctl", "start", "robotcore.service"] in calls


def test_pid_config_save_updates_active_and_named_profile_atomically():
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        manager = make_control_manager(root)
        config = manager.handle({"action": "pid-config"})["config"]
        config["profile_name"] = "pool_tune_01"
        config["configured"] = True
        config["inner_kp"][0] = 12.5

        saved = manager.handle({"action": "pid-save", "config": config})

        assert saved["accepted"] is True
        assert manager.handle({"action": "pid-config"})["config"] == config
        profile = json.loads(
            (root / "pid/profiles/pool_tune_01.json").read_text(encoding="utf-8")
        )
        assert profile == config


def test_task_files_are_listed_read_and_written_by_name():
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        manager = make_control_manager(root)

        tasks = manager.handle({"action": "task-list"})["tasks"]
        assert {task["name"] for task in tasks} == {
            "pose_hold",
            "station_hold",
            "station_hold_fast",
            "spatial_figure_eight",
        }
        assert "record_topics" not in {task["name"] for task in tasks}

        task = manager.handle({"action": "task-get", "name": "pose_hold"})["task"]
        task["name"] = "altitude_hold_test"
        task["label"] = "Altitude hold test"
        task["trajectory"]["manual_vertical_speed_mps"] = 0.1
        manager.handle({"action": "task-save", "task": task})

        saved = manager.handle(
            {"action": "task-get", "name": "altitude_hold_test"}
        )["task"]
        assert saved["trajectory"]["manual_vertical_speed_mps"] == 0.1
