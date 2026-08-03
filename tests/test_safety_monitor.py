"""Low-level board and operator abort behavior without a live ROS graph."""

from pathlib import Path
from types import SimpleNamespace

import pytest

from robotcore_runtime.safety_monitor import SafetyMonitor


ROOT = Path(__file__).resolve().parents[1]


def make_monitor():
    monitor = object.__new__(SafetyMonitor)
    monitor.test_steady_ns = 1_000_000_000
    monitor.operator_abort_active = False
    monitor.operator_abort_reason = ""
    monitor.abort_active = True
    monitor.board_status_ns = None
    monitor.board_connected = False
    monitor.board_heartbeat_ok = False
    monitor.board_failsafe_active = False
    parameters = {
        "board_status_timeout_s": 0.50,
    }
    monitor.get_parameter = lambda name: SimpleNamespace(value=parameters[name])
    monitor.steady_now_ns = lambda: monitor.test_steady_ns
    monitor.published = []
    monitor.publish_event = lambda level, code, message: monitor.published.append(
        (level, code, message, monitor.abort_active)
    )
    return monitor


def board_status(**overrides):
    values = {
        "connected": True,
        "heartbeat_ok": True,
        "failsafe_active": False,
    }
    values.update(overrides)
    return SimpleNamespace(**values)


def test_board_status_is_required_by_default():
    monitor = make_monitor()

    code, message = monitor.update_abort_state(monitor.test_steady_ns)

    assert code == "BOARD_STATUS_MISSING"
    assert "missing" in message
    assert monitor.abort_active is True


def test_healthy_board_clears_dynamic_board_abort():
    monitor = make_monitor()
    monitor.on_board_status(board_status())

    code, _message = monitor.update_abort_state(monitor.test_steady_ns)

    assert code == "ABORT_CLEAR"
    assert monitor.abort_active is False


@pytest.mark.parametrize(
    ("overrides", "expected_code"),
    [
        ({"connected": False}, "BOARD_DISCONNECTED"),
        ({"heartbeat_ok": False}, "BOARD_HEARTBEAT_LOST"),
        ({"failsafe_active": True}, "BOARD_FAILSAFE_ACTIVE"),
    ],
)
def test_each_board_safety_signal_aborts(overrides, expected_code):
    monitor = make_monitor()
    monitor.on_board_status(board_status(**overrides))

    code, _message = monitor.update_abort_state(monitor.test_steady_ns)

    assert code == expected_code
    assert monitor.abort_active is True


def test_missing_heartbeat_is_reported_before_aggregate_failsafe():
    monitor = make_monitor()
    monitor.on_board_status(
        board_status(heartbeat_ok=False, failsafe_active=True)
    )

    code, _message = monitor.update_abort_state(monitor.test_steady_ns)

    assert code == "BOARD_HEARTBEAT_LOST"
    assert monitor.abort_active is True


def test_board_status_freshness_uses_monotonic_time():
    monitor = make_monitor()
    monitor.on_board_status(board_status())
    monitor.test_steady_ns += 510_000_000

    code, _message = monitor.update_abort_state(monitor.test_steady_ns)

    assert code == "BOARD_STATUS_STALE"
    assert monitor.abort_active is True


def test_operator_clear_cannot_clear_live_board_fault():
    monitor = make_monitor()
    monitor.on_board_status(board_status(connected=False))
    request = SimpleNamespace(abort=False, reason="")
    response = SimpleNamespace()

    monitor.handle_abort(request, response)

    assert response.accepted is True
    assert response.abort_active is True
    assert "disconnected" in response.message
    assert monitor.published[-1][1] == "BOARD_DISCONNECTED"


def test_edge_launch_does_not_disable_physical_board_requirement():
    source = (
        ROOT / "ros_ws/src/robotcore_runtime/robotcore_runtime/safety_monitor.py"
    ).read_text(encoding="utf-8")

    assert 'declare_parameter("require_board_status"' not in source
    assert "self.abort_active = True" in source


def test_operator_abort_remains_latched_with_a_healthy_board():
    monitor = make_monitor()
    monitor.on_board_status(board_status())
    request = SimpleNamespace(abort=True, reason="operator stop")
    response = SimpleNamespace()

    monitor.handle_abort(request, response)

    assert response.accepted is True
    assert response.abort_active is True
    assert response.message == "operator stop"
    assert monitor.published[-1][1] == "ABORT_ACTIVE"
