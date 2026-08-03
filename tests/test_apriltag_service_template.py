"""Regression checks for the production AprilTag-map launch wiring."""

import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SERVICE = ROOT / "host_manager" / "systemd" / "robotcore.service"
ARGUS_DROP_IN = ROOT / "host_manager" / "systemd" / "robotcore-argus.conf"
EDGE_ENV = ROOT / "host_manager" / "systemd" / "edge.env.example"
HOST_MANAGER_CONFIG = ROOT / "host_manager" / "config" / "host-manager.example.json"


def test_robot_service_requires_and_passes_the_managed_apriltag_map():
    service = SERVICE.read_text(encoding="utf-8")

    assert "ExecStartPre=/usr/bin/test -r ${ROBOTCORE_APRILTAG_MAP_FILE}" in service
    assert 'apriltag_tag_map_file:="${ROBOTCORE_APRILTAG_MAP_FILE}"' in service
    assert not (ROOT / "host_manager/systemd/robotcore-tag-vio.conf").exists()


def test_robot_service_exposes_only_the_argus_socket_to_the_zed_runtime():
    service = SERVICE.read_text(encoding="utf-8")

    assert "PrivateTmp=true" in service
    assert "BindReadOnlyPaths=/tmp/argus_socket" in service
    assert "BindReadOnlyPaths=/tmp/imu_daemon.sock" in service
    assert "After=network-online.target dev-robotcore-aboard.device nvargus-daemon.service" in service
    assert "Wants=network-online.target nvargus-daemon.service" in service


def test_argus_drop_in_preserves_tmp_isolation_except_for_the_ipc_socket():
    drop_in = ARGUS_DROP_IN.read_text(encoding="utf-8")

    assert "After=nvargus-daemon.service" in drop_in
    assert "Wants=nvargus-daemon.service" in drop_in
    assert "BindReadOnlyPaths=/tmp/argus_socket" in drop_in
    assert "BindReadOnlyPaths=/tmp/imu_daemon.sock" in drop_in
    assert "SupplementaryGroups=imu" in drop_in
    assert "PrivateTmp=false" not in drop_in


def test_example_edge_environment_uses_the_managed_map_default():
    edge_env = EDGE_ENV.read_text(encoding="utf-8")
    configured_map = json.loads(HOST_MANAGER_CONFIG.read_text(encoding="utf-8"))[
        "apriltag_map"
    ]["path"]

    assert f"ROBOTCORE_APRILTAG_MAP_FILE={configured_map}" in edge_env
