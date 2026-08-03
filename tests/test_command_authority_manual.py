"""Manual command-authority behavior independent of ROS graph construction."""

from types import SimpleNamespace

import numpy as np

from robotcore_control.command_authority_node import CommandAuthorityNode


def make_authority(*, enable: bool, candidate_age_s: float = 0.0):
    authority = object.__new__(CommandAuthorityNode)
    now_ns = 1_000_000_000
    authority.test_steady_ns = now_ns
    authority.test_ros_ns = 9_000_000_000
    authority.selected_source = "manual"
    authority.armed = True
    authority.arm_generation = 0
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
    authority.manual_command_was_active = False
    authority.last_output = np.zeros(8, dtype=np.float64)
    authority.last_tick_ns = now_ns - 10_000_000
    parameters = {
        "candidate_timeout_s": 0.10,
        "safety_heartbeat_timeout_s": 0.25,
        "automatic_command_limit": 0.15,
        "automatic_slew_rate_per_s": 10.0,
    }
    authority.get_parameter = lambda name: SimpleNamespace(value=parameters[name])
    authority.steady_now_ns = lambda: authority.test_steady_ns
    authority.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=authority.test_ros_ns)
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


def test_selected_manual_explicit_release_clears_stream_latch_without_fault():
    authority = make_authority(enable=True)

    authority.tick()
    assert authority.manual_command_was_active is True

    authority.test_steady_ns += 10_000_000
    authority.candidates["manual"].enable = False
    authority.candidate_ns["manual"] = authority.test_steady_ns
    authority.tick()

    assert authority.armed is True
    assert authority.fault_latched is False
    assert authority.manual_command_was_active is False
    assert "neutral" in authority.message
    assert authority.published[-1] == ([0.0] * 8, False)


def test_selected_manual_timeout_trips_and_cannot_resume_from_a_late_frame():
    authority = make_authority(enable=True)

    authority.tick()
    assert authority.manual_command_was_active is True

    authority.test_steady_ns += 200_000_000
    authority.tick()

    assert authority.armed is False
    assert authority.fault_latched is True
    assert authority.fault_code == "MANUAL_LINK_LOST"
    assert "missing or stale" in authority.message
    assert authority.published[-1] == ([0.0] * 8, False)

    # A delayed old frame can become fresh at the ROS receiver, but it cannot
    # revive output because loss of the preceding stream disarmed authority.
    authority.candidate_ns["manual"] = authority.test_steady_ns
    authority.tick()

    assert authority.armed is False
    assert authority.fault_code == "MANUAL_LINK_LOST"
    assert authority.published[-1] == ([0.0] * 8, False)


def test_manual_candidate_survives_short_producer_jitter():
    authority = make_authority(enable=True, candidate_age_s=0.08)

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
    assert authority.active_source == "manual"
    assert authority.manual_command_was_active is True

    authority.test_steady_ns += 10_000_000
    authority.candidates["manual"].enable = False
    authority.candidate_ns["manual"] = authority.test_steady_ns
    authority.candidate_ns["pid"] = authority.test_steady_ns
    authority.tick()

    assert authority.selected_source == "pid"
    assert authority.active_source == "pid"
    assert authority.armed is True
    assert authority.fault_latched is False
    assert authority.manual_command_was_active is False
    assert authority.published[-1] == ([0.02] * 8, True)


def test_pid_does_not_resume_when_active_manual_override_times_out():
    authority = make_authority(enable=True)
    authority.selected_source = "pid"
    authority.candidates["pid"] = SimpleNamespace(
        enable=True,
        source="pid_controller",
        normalized=[0.02] * 8,
    )
    authority.candidate_ns["pid"] = authority.test_steady_ns
    authority.automatic_prearm_failure = lambda _now_ns: ""

    authority.tick()
    assert authority.active_source == "manual"

    authority.test_steady_ns += 200_000_000
    authority.candidate_ns["pid"] = authority.test_steady_ns
    authority.tick()

    assert authority.selected_source == "pid"
    assert authority.active_source == "manual"
    assert authority.armed is False
    assert authority.fault_latched is True
    assert authority.fault_code == "MANUAL_LINK_LOST"
    assert authority.published[-1] == ([0.0] * 8, False)


def test_missing_active_manual_override_trips_instead_of_resuming_pid():
    authority = make_authority(enable=True)
    authority.selected_source = "pid"
    authority.candidates["pid"] = SimpleNamespace(
        enable=True,
        source="pid_controller",
        normalized=[0.02] * 8,
    )
    authority.candidate_ns["pid"] = authority.test_steady_ns
    authority.automatic_prearm_failure = lambda _now_ns: ""
    authority.tick()

    authority.candidates.pop("manual")
    authority.candidate_ns.pop("manual")
    authority.tick()

    assert authority.armed is False
    assert authority.fault_code == "MANUAL_LINK_LOST"
    assert "missing or stale" in authority.message
    assert authority.published[-1] == ([0.0] * 8, False)


def test_invalid_active_manual_override_trips_instead_of_resuming_pid():
    authority = make_authority(enable=True)
    authority.selected_source = "pid"
    authority.candidates["pid"] = SimpleNamespace(
        enable=True,
        source="pid_controller",
        normalized=[0.02] * 8,
    )
    authority.candidate_ns["pid"] = authority.test_steady_ns
    authority.automatic_prearm_failure = lambda _now_ns: ""
    authority.tick()

    authority.candidates["manual"].normalized[0] = float("nan")
    authority.candidate_ns["manual"] = authority.test_steady_ns
    authority.tick()

    assert authority.armed is False
    assert authority.fault_code == "MANUAL_LINK_LOST"
    assert "eight finite values" in authority.message
    assert authority.published[-1] == ([0.0] * 8, False)


def test_manual_candidate_cannot_drive_without_arm():
    authority = make_authority(enable=True)
    authority.armed = False

    authority.tick()

    assert authority.armed is False
    assert authority.active_source == "manual"
    assert authority.published[-1] == ([0.0] * 8, False)


def test_lb_common_safety_failure_never_publishes_enabled_output():
    authority = make_authority(enable=True)
    authority.abort_active = True

    authority.tick()

    assert authority.armed is False
    assert authority.fault_latched is True
    assert authority.published[-1] == ([0.0] * 8, False)


def test_manual_arm_request_uses_fresh_candidate_and_safety_heartbeat():
    authority = make_authority(enable=True)
    authority.armed = False
    request = SimpleNamespace(source="manual", arm=True, clear_fault=False)
    response = SimpleNamespace()

    authority.on_set_authority(request, response)

    assert response.accepted is True
    assert response.armed is True
    assert authority.armed is True
    assert authority.arm_generation == 1
    assert authority.manual_command_was_active is True


def test_repeated_arm_request_does_not_advance_generation():
    authority = make_authority(enable=True)
    authority.armed = False
    request = SimpleNamespace(source="manual", arm=True, clear_fault=False)

    authority.on_set_authority(request, SimpleNamespace())
    first_generation = authority.arm_generation
    authority.on_set_authority(request, SimpleNamespace())

    assert first_generation == 1
    assert authority.arm_generation == first_generation


def test_only_successful_disarmed_to_armed_transition_advances_generation():
    authority = make_authority(enable=True)
    authority.armed = False
    arm = SimpleNamespace(source="manual", arm=True, clear_fault=False)
    disarm = SimpleNamespace(source="manual", arm=False, clear_fault=False)

    authority.on_set_authority(arm, SimpleNamespace())
    authority.on_set_authority(disarm, SimpleNamespace())
    authority.on_set_authority(arm, SimpleNamespace())

    assert authority.armed is True
    assert authority.arm_generation == 2


def test_rejected_arm_request_does_not_advance_generation():
    authority = make_authority(enable=False)
    authority.armed = False
    request = SimpleNamespace(source="manual", arm=True, clear_fault=False)
    response = SimpleNamespace()

    authority.on_set_authority(request, response)

    assert response.accepted is False
    assert authority.armed is False
    assert authority.arm_generation == 0


def test_deadman_release_does_not_advance_arm_generation():
    authority = make_authority(enable=False)
    authority.arm_generation = 7

    authority.tick()

    assert authority.armed is True
    assert authority.published[-1] == ([0.0] * 8, False)
    assert authority.arm_generation == 7


def test_ros_clock_jump_does_not_extend_manual_candidate_freshness():
    authority = make_authority(enable=True)

    authority.tick()
    authority.test_ros_ns -= 8_000_000_000
    authority.test_steady_ns += 200_000_000

    authority.tick()

    assert authority.armed is False
    assert authority.fault_latched is True
    assert authority.fault_code == "MANUAL_LINK_LOST"
    assert authority.published[-1] == ([0.0] * 8, False)


def test_candidate_receipt_uses_monotonic_not_ros_clock():
    authority = make_authority(enable=True)
    authority.candidates.clear()
    authority.candidate_ns.clear()
    authority.test_steady_ns = 1_500_000_000
    authority.test_ros_ns = -20_000_000_000
    message = SimpleNamespace(
        enable=True,
        source="web_operator",
        normalized=[0.2] * 8,
    )

    authority.on_candidate("manual", message)

    assert authority.candidate_ns["manual"] == 1_500_000_000


def test_board_failure_code_is_preserved_in_authority_fault():
    authority = make_authority(enable=True)
    event = SimpleNamespace(
        abort_active=True,
        code="BOARD_HEARTBEAT_LOST",
        message="low-level board heartbeat is not healthy",
    )

    authority.on_safety(event)

    assert authority.armed is False
    assert authority.fault_latched is True
    assert authority.fault_code == "BOARD_HEARTBEAT_LOST"


def test_board_failure_while_disarmed_blocks_prearm_without_latching_fault():
    authority = make_authority(enable=True)
    authority.armed = False
    event = SimpleNamespace(
        abort_active=True,
        code="BOARD_HEARTBEAT_LOST",
        message="low-level board heartbeat is not healthy",
    )

    authority.on_safety(event)

    assert authority.abort_active is True
    assert authority.armed is False
    assert authority.fault_latched is False
    assert authority.fault_code == ""
    assert authority.message == "armed manual"


def test_rejected_authority_reason_is_exposed_in_status_message():
    authority = make_authority(enable=False)
    response = SimpleNamespace()

    authority.reject(response, "pool bounds are not configured")

    assert response.accepted is False
    assert response.message == "pool bounds are not configured"
    assert authority.message == "pool bounds are not configured"
