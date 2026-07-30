"""Allowlisted Unix-socket host-management daemon.

This module intentionally depends only on the Python standard library.  It is
not a ROS node and it exposes no network listener.
"""

from __future__ import annotations

import argparse
import grp
import json
import os
import socketserver
import stat
import subprocess
import time
from pathlib import Path
from typing import Any


MAX_REQUEST_BYTES = 4096
MAX_LOG_LINES = 120
ALLOWED_ACTIONS = {
    "status",
    "devices",
    "rosbag-status",
    "logs",
    "start",
    "stop",
    "restart",
    "maintenance-status",
    "apriltag-map",
    "apriltag-upsert",
    "apriltag-delete",
}


class HostManager:
    """Collect local health and execute a deliberately small service allowlist."""

    def __init__(self, config: dict[str, Any]):
        self.config = config
        self.services = config.get("services", {})

    @staticmethod
    def _run(command: list[str], timeout_s: float = 8.0) -> dict[str, Any]:
        try:
            completed = subprocess.run(
                command,
                check=False,
                capture_output=True,
                text=True,
                timeout=timeout_s,
            )
        except (OSError, subprocess.TimeoutExpired) as exc:
            return {"ok": False, "returncode": None, "output": "", "error": str(exc)}
        return {
            "ok": completed.returncode == 0,
            "returncode": completed.returncode,
            "output": (completed.stdout or completed.stderr).strip(),
            "error": "",
        }

    def _service_name(self, requested: str) -> str:
        name = self.services.get(requested)
        if not isinstance(name, str) or not name.endswith(".service"):
            raise ValueError("unknown managed service")
        return name

    def service_status(self) -> dict[str, Any]:
        result: dict[str, Any] = {}
        for role in sorted(self.services):
            name = self._service_name(role)
            result[role] = self._run(
                ["systemctl", "show", name, "--property=ActiveState,SubState,MainPID", "--no-page"]
            )
        return result

    def device_status(self) -> list[dict[str, Any]]:
        report = []
        for device in self.config.get("devices", []):
            path = Path(str(device.get("path", "")))
            try:
                mode = path.stat().st_mode
                exists = True
                character_device = stat.S_ISCHR(mode)
            except OSError:
                exists = False
                character_device = False
            report.append(
                {
                    "name": str(device.get("name", path.name)),
                    "path": str(path),
                    "required": bool(device.get("required", False)),
                    "exists": exists,
                    "character_device": character_device,
                    "readable": os.access(path, os.R_OK) if exists else False,
                    "writable": os.access(path, os.W_OK) if exists else False,
                }
            )
        return report

    def rosbag_status(self) -> dict[str, Any]:
        config = self.config.get("rosbag", {})
        root = Path(str(config.get("run_root", "/var/lib/robotcore/runs")))
        stale_after = max(1, int(config.get("stale_after_seconds", 15)))
        candidates: list[Path] = []
        if root.is_dir():
            # metadata.yaml is normally finalised on recorder shutdown, while
            # the active sqlite/MCAP file advances during recording.  Monitor
            # both so a healthy live bag is not reported stale.
            candidates = [
                path
                for pattern in ("metadata.yaml", "*.db3", "*.mcap")
                for path in root.rglob(pattern)
                if "rosbag2" in path.parts
            ]
        latest = max(candidates, key=lambda p: p.stat().st_mtime, default=None)
        if latest is None:
            return {
                "root": str(root),
                "recording_seen": False,
                "fresh": False,
                "reason": "no rosbag metadata found",
            }
        age = max(0.0, time.time() - latest.stat().st_mtime)
        return {
            "root": str(root),
            "recording_seen": True,
            "fresh": age <= stale_after,
            "latest_artifact": str(latest),
            "age_seconds": round(age, 3),
            "stale_after_seconds": stale_after,
        }

    def logs(self, role: str) -> dict[str, Any]:
        return self._run(
            [
                "journalctl",
                "--unit",
                self._service_name(role),
                "--lines",
                str(MAX_LOG_LINES),
                "--no-pager",
                "--output=short-iso",
            ]
        )

    def lifecycle(self, action: str, role: str) -> dict[str, Any]:
        if action not in {"start", "stop", "restart"}:
            raise ValueError("unsupported lifecycle action")
        service = self._service_name(role)
        result = self._run(["systemctl", action, service])
        result["service"] = service
        result["status"] = self._run(
            ["systemctl", "show", service, "--property=LoadState,ActiveState,SubState,Result", "--no-page"]
        )
        # A failed lifecycle command commonly has no stdout/stderr.  Include a
        # short journal excerpt so the browser can report the actual edge
        # failure (missing device, environment file, workspace, etc.).
        if not result["ok"]:
            result["recent_logs"] = self._run(
                [
                    "journalctl",
                    "--unit",
                    service,
                    "--lines",
                    "12",
                    "--no-pager",
                    "--output=short-iso",
                ]
            )
        return result

    def _apriltag_config(self) -> dict[str, Any]:
        config = self.config.get("apriltag_map", {})
        if not isinstance(config, dict):
            raise ValueError("apriltag_map configuration is missing")
        path = Path(str(config.get("path", "")))
        if not path.is_absolute() or not path.name:
            raise ValueError("apriltag_map.path must be an absolute file path")
        return {**config, "path": path}

    @staticmethod
    def _tag_map_document(path: Path) -> dict[str, Any]:
        if not path.exists():
            return {"schema_version": 1, "frame": "map", "tags": {}}
        document = json.loads(path.read_text(encoding="utf-8"))
        if not isinstance(document, dict) or not isinstance(document.get("tags"), dict):
            raise ValueError("managed AprilTag map must be a JSON object with tags")
        document.setdefault("schema_version", 1)
        document.setdefault("frame", "map")
        return document

    @staticmethod
    def _finite_vector(raw: Any, label: str, lower: float, upper: float) -> list[float]:
        try:
            values = [float(value) for value in raw]
        except (TypeError, ValueError) as exc:
            raise ValueError(f"{label} must contain three numbers") from exc
        if len(values) != 3 or any(not (lower <= value <= upper) for value in values):
            raise ValueError(f"{label} values must be finite and within [{lower}, {upper}]")
        return values

    @staticmethod
    def _validated_tag_id(raw_id: Any) -> int:
        if isinstance(raw_id, bool):
            raise ValueError("tag id must be an integer")
        try:
            tag_id = int(raw_id)
        except (TypeError, ValueError) as exc:
            raise ValueError("tag id must be an integer") from exc
        if not 0 <= tag_id <= 65535 or str(tag_id) != str(raw_id).strip():
            raise ValueError("tag id must be an unsigned integer from 0 to 65535")
        return tag_id

    def _validated_tag(self, tag: Any) -> tuple[int, dict[str, Any]]:
        if not isinstance(tag, dict):
            raise ValueError("tag must be an object")
        tag_id = self._validated_tag_id(tag.get("id"))
        position_m = self._finite_vector(tag.get("position_m"), "position_m", -100.0, 100.0)
        rpy_deg = self._finite_vector(tag.get("rpy_deg"), "rpy_deg", -360.0, 360.0)
        try:
            size_m = float(tag.get("size_m"))
        except (TypeError, ValueError) as exc:
            raise ValueError("size_m must be a number") from exc
        if not 0.01 <= size_m <= 2.0:
            raise ValueError("size_m must be between 0.01 and 2.0 metres")
        return tag_id, {"position_m": position_m, "rpy_deg": rpy_deg, "size_m": size_m}

    @staticmethod
    def _write_apriltag_map(path: Path, document: dict[str, Any]) -> None:
        """Atomically replace the only host-managed map file."""

        path.parent.mkdir(mode=0o750, parents=True, exist_ok=True)
        temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
        payload = json.dumps(document, indent=2, sort_keys=True) + "\n"
        try:
            with temporary.open("w", encoding="utf-8") as handle:
                handle.write(payload)
                handle.flush()
                os.fsync(handle.fileno())
            os.replace(temporary, path)
        finally:
            if temporary.exists():
                temporary.unlink()

    @staticmethod
    def _tag_list(document: dict[str, Any]) -> list[dict[str, Any]]:
        tags = document.get("tags", {})
        result = []
        for raw_id in sorted(tags, key=lambda value: int(value)):
            definition = tags[raw_id]
            result.append({"id": int(raw_id), **definition})
        return result

    def apriltag_map(self) -> dict[str, Any]:
        config = self._apriltag_config()
        document = self._tag_map_document(config["path"])
        return {
            "accepted": True,
            "path": str(config["path"]),
            "web_edit_enabled": bool(config.get("web_edit_enabled", False)),
            "tags": self._tag_list(document),
        }

    def upsert_apriltag(self, tag: Any, replace_tag_id: Any = None) -> dict[str, Any]:
        config = self._apriltag_config()
        if not bool(config.get("web_edit_enabled", False)):
            raise ValueError("AprilTag web editing is disabled by host-manager policy")
        tag_id, definition = self._validated_tag(tag)
        previous_tag_id = (
            tag_id if replace_tag_id is None else self._validated_tag_id(replace_tag_id)
        )
        path = config["path"]
        document = self._tag_map_document(path)
        if previous_tag_id != tag_id:
            if str(previous_tag_id) not in document["tags"]:
                raise ValueError(f"tag {previous_tag_id} does not exist in the managed AprilTag map")
            if str(tag_id) in document["tags"]:
                raise ValueError(f"tag {tag_id} already exists in the managed AprilTag map")
            del document["tags"][str(previous_tag_id)]
        document["tags"][str(tag_id)] = definition
        self._write_apriltag_map(path, document)
        operation = f"renamed tag {previous_tag_id} to {tag_id}" if previous_tag_id != tag_id else f"saved tag {tag_id}"
        return {
            "accepted": True,
            "message": f"{operation}; localization will reload the map file",
            "path": str(path),
            "tag": {"id": tag_id, **definition},
            "previous_tag_id": previous_tag_id,
            "tags": self._tag_list(document),
        }

    def delete_apriltag(self, raw_tag_id: Any) -> dict[str, Any]:
        """Delete one tag definition while retaining the prior map revision."""

        config = self._apriltag_config()
        if not bool(config.get("web_edit_enabled", False)):
            raise ValueError("AprilTag web editing is disabled by host-manager policy")
        tag_id = self._validated_tag_id(raw_tag_id)
        path = config["path"]
        document = self._tag_map_document(path)
        if str(tag_id) not in document["tags"]:
            raise ValueError(f"tag {tag_id} does not exist in the managed AprilTag map")
        del document["tags"][str(tag_id)]
        self._write_apriltag_map(path, document)
        return {
            "accepted": True,
            "message": f"deleted tag {tag_id}; localization will reload the map file",
            "path": str(path),
            "tag_id": tag_id,
            "tags": self._tag_list(document),
        }

    def handle(self, request: dict[str, Any]) -> dict[str, Any]:
        action = str(request.get("action", ""))
        if action not in ALLOWED_ACTIONS:
            raise ValueError("action is not permitted")
        if action == "status":
            return {"services": self.service_status(), "devices": self.device_status(), "rosbag": self.rosbag_status()}
        if action == "devices":
            return {"devices": self.device_status()}
        if action == "rosbag-status":
            return self.rosbag_status()
        if action == "maintenance-status":
            return dict(self.config.get("maintenance", {}))
        if action == "apriltag-map":
            return self.apriltag_map()
        if action == "apriltag-upsert":
            return self.upsert_apriltag(request.get("tag"), request.get("replace_tag_id"))
        if action == "apriltag-delete":
            return self.delete_apriltag(request.get("tag_id"))
        role = str(request.get("service", ""))
        if action == "logs":
            return self.logs(role)
        return self.lifecycle(action, role)


def load_config(path: Path) -> dict[str, Any]:
    config = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(config, dict) or config.get("schema_version") != 1:
        raise ValueError("unsupported host-manager configuration")
    if not isinstance(config.get("services"), dict):
        raise ValueError("configuration must define managed services")
    return config


class RequestHandler(socketserver.StreamRequestHandler):
    """One JSON request per local socket connection."""

    def handle(self) -> None:
        raw = self.rfile.readline(MAX_REQUEST_BYTES + 1)
        if len(raw) > MAX_REQUEST_BYTES:
            response: dict[str, Any] = {"ok": False, "error": "request too large"}
        else:
            try:
                request = json.loads(raw.decode("utf-8"))
                if not isinstance(request, dict):
                    raise ValueError("request must be an object")
                response = {"ok": True, "result": self.server.manager.handle(request)}  # type: ignore[attr-defined]
            except (UnicodeDecodeError, ValueError, json.JSONDecodeError) as exc:
                response = {"ok": False, "error": str(exc)}
        self.wfile.write((json.dumps(response, sort_keys=True) + "\n").encode("utf-8"))


class UnixServer(socketserver.ThreadingUnixStreamServer):
    daemon_threads = True
    allow_reuse_address = True


def serve(config_path: Path, socket_path: Path) -> None:
    config = load_config(config_path)
    socket_path.parent.mkdir(mode=0o750, parents=True, exist_ok=True)
    if socket_path.exists():
        socket_path.unlink()
    with UnixServer(str(socket_path), RequestHandler) as server:
        server.manager = HostManager(config)  # type: ignore[attr-defined]
        group = grp.getgrnam(str(config.get("socket_group", "robotops")))
        os.chown(socket_path, 0, group.gr_gid)
        os.chmod(socket_path, 0o660)
        server.serve_forever(poll_interval=0.5)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="RobotCore local host-management daemon")
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--socket", type=Path, default=Path("/run/robotcore/host-manager.sock"))
    args = parser.parse_args(argv)
    serve(args.config, args.socket)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
