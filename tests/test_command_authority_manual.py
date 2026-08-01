"""Manual command-authority behavior independent of ROS graph construction."""

from types import SimpleNamespace

import numpy as np

from eup_control.command_authority_node import CommandAuthorityNode


def make_authority(*, enable: bool, candidate_age_s: float = 0.0):
    authority = object.__new__(CommandAuthorityNode)
    now_ns = 1_000_000_000
    authority.selected_source = "manual"
    authority.armed = True
    authority.abort_active = False
    authority.fault_latched = False
    authority.fault_code = ""
    authority.message = "armed manual"
    authority.candidates = {
        "manual": SimpleNamespace(
            enable=enable,
            source="web_operator",
            normalized=[0.1] * 8,
        )
    }
    authority.candidate_ns = {
        "manual": now_ns - int(candidate_age_s * 1e9),
    }
    authority.safety_ns = now_ns
    authority.body = None
    authority.body_ns = None
    authority.target = None
    authority.target_ns = None
    authority.absolute_localization_seen = False
    authority.manual_override_was_active = False
    authority.last_output = np.zeros(8, dtype=np.float64)
    authority.last_tick_ns = now_ns - 10_000_000
    parameters = {
        "candidate_timeout_s": 0.10,
        "manual_candidate_timeout_s": 0.60,
        "safety_heartbeat_timeout_s": 0.25,
    }
    authority.get_parameter = lambda name: SimpleNamespace(value=parameters[name])
    authority.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=now_ns)
    )
    authority.published = []
    authority.publish_command = (
        lambda now, values, enabled: authority.published.append(
            (list(values), bool(enabled))
        )
    )
    authority.publish_status = lambda now, timestamp: None
    return authority


def test_manual_enabled_candidate_does_not_require_tracking_inputs():
    authority = make_authority(enable=True)

    authority.tick()

    assert authority.armed is True
    assert authority.fault_latched is False
    assert authority.published[-1] == ([0.1] * 8, True)


def test_manual_deadman_release_is_neutral_idle_not_latched_fault():
    authority = make_authority(enable=False)

    authority.tick()

    assert authority.armed is True
    assert authority.fault_latched is False
    assert authority.fault_code == ""
    assert "neutral" in authority.message
    assert authority.published[-1] == ([0.0] * 8, False)


def test_stale_manual_candidate_is_neutral_idle_not_latched_fault():
    authority = make_authority(enable=True, candidate_age_s=0.8)

    authority.tick()

    assert authority.armed is True
    assert authority.fault_latched is False
    assert authority.published[-1] == ([0.0] * 8, False)


def test_manual_candidate_survives_short_ros_publisher_jitter():
    authority = make_authority(enable=True, candidate_age_s=0.5)

    authority.tick()

    assert authority.active_source == "manual"
    assert authority.published[-1] == ([0.1] * 8, True)


def test_lb_manual_candidate_overrides_pid_without_changing_selection():
    authority = make_authority(enable=True)
    authority.selected_source = "pid"
    authority.candidates["pid"] = SimpleNamespace(
        enable=True,
        source="pid_controller",
        normalized=[0.02] * 8,
    )
    authority.candidate_ns["pid"] = 1_000_000_000
    authority.automatic_prearm_failure = lambda _now_ns: ""

    authority.tick()

    assert authority.selected_source == "pid"
    assert authority.active_source == "manual"
    assert authority.published[-1] == ([0.1] * 8, True)
    assert "manual LB override" in authority.message


def test_pid_resumes_when_lb_manual_candidate_is_disabled():
    authority = make_authority(enable=False)
    authority.selected_source = "pid"
    authority.candidates["pid"] = SimpleNamespace(
        enable=True,
        source="pid_controller",
        normalized=[0.02] * 8,
    )
    authority.candidate_ns["pid"] = 1_000_000_000
    authority.automatic_prearm_failure = lambda _now_ns: ""
    parameters = {
        "candidate_timeout_s": 0.10,
        "manual_candidate_timeout_s": 0.60,
        "safety_heartbeat_timeout_s": 0.25,
        "automatic_command_limit": 0.15,
        "automatic_slew_rate_per_s": 10.0,
    }
    authority.get_parameter = lambda name: SimpleNamespace(value=parameters[name])

    authority.tick()

    assert authority.selected_source == "pid"
    assert authority.active_source == "pid"
    assert authority.published[-1] == ([0.02] * 8, True)


def test_lb_drives_directly_without_select_and_arm():
    authority = make_authority(enable=True)
    authority.armed = False

    authority.tick()

    assert authority.armed is False
    assert authority.active_source == "manual"
    assert authority.published[-1] == ([0.1] * 8, True)


def test_lb_common_safety_failure_never_publishes_enabled_output():
    authority = make_authority(enable=True)
    authority.armed = False
    authority.abort_active = True

    authority.tick()

    assert authority.armed is False
    assert authority.fault_latched is True
    assert authority.published[-1] == ([0.0] * 8, False)


def test_direct_lb_message_clears_when_manual_candidate_is_disabled():
    authority = make_authority(enable=False)
    authority.armed = False
    authority.message = "manual LB direct control"
    authority.manual_override_was_active = True

    authority.tick()

    assert authority.message == "disarmed"
    assert authority.published[-1] == ([0.0] * 8, False)


def test_rejected_authority_reason_is_exposed_in_status_message():
    authority = make_authority(enable=False)
    response = SimpleNamespace()

    authority.reject(response, "pool bounds are not configured")

    assert response.accepted is False
    assert response.message == "pool bounds are not configured"
    assert authority.message == "pool bounds are not configured"
