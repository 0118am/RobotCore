from pathlib import Path
import sys

import numpy as np


PACKAGE_ROOT = Path(__file__).resolve().parents[1] / "ros_ws" / "src" / "robotcore_control"
sys.path.insert(0, str(PACKAGE_ROOT))

import robotcore_control.thruster_allocation as allocation_module  # noqa: E402
from robotcore_control.control_math import (  # noqa: E402
    altitude_level_pwm_commands,
    altitude_station_pwm_commands,
)
from robotcore_control.thruster_allocation import ThrusterAllocator  # noqa: E402


CONFIG = (
    Path(__file__).resolve().parents[1]
    / "ros_ws"
    / "src"
    / "robotcore_control"
    / "config"
    / "real_pool_thrusters.yaml"
)


def test_validated_vector_model_is_full_rank_and_measured():
    allocator = ThrusterAllocator.from_yaml(CONFIG)
    assert allocator.rank == 6
    assert np.isfinite(allocator.condition)
    assert allocator.measured is True
    assert allocator.vector_mode is True
    assert len(allocator.config_hash) == 16


def test_allocator_tracks_feasible_wrench_and_returns_eight_commands():
    allocator = ThrusterAllocator.from_yaml(CONFIG)
    source_commands = [0.18, -0.16, 0.14, -0.12, 0.20, -0.18, 0.16, -0.14]
    target = allocator.wrench_for_commands(source_commands)
    result = allocator.allocate(target)
    assert len(result.commands) == 8
    assert all(-0.4 <= value <= 0.4 for value in result.commands)
    assert result.residual < 0.05
    assert result.saturation_fraction == 0.0


def test_allocator_bounds_infeasible_wrench():
    allocator = ThrusterAllocator.from_yaml(CONFIG)
    result = allocator.allocate([10000.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    assert all(-0.4 <= value <= 0.4 for value in result.commands)
    assert result.saturation_fraction > 0.0
    assert result.residual > 1000.0


def test_allocator_solves_inside_live_web_pwm_limit_without_post_clipping():
    allocator = ThrusterAllocator.from_yaml(CONFIG)
    command_limit = 0.2
    source_commands = [0.18, -0.16, 0.14, -0.12, 0.20, -0.18, 0.16, -0.14]
    target = allocator.wrench_for_commands(source_commands)

    result = allocator.allocate(target, command_limit=command_limit)
    achieved = allocator.wrench_for_commands(result.commands)

    assert max(abs(value) for value in result.commands) <= command_limit + 1e-9
    assert np.isclose(
        result.residual,
        np.linalg.norm(allocator.axis_weights * (target - achieved)),
        atol=1e-9,
    )
    assert result.residual < 0.01


def test_live_pwm_limit_reports_real_saturation_and_handles_zero_authority():
    allocator = ThrusterAllocator.from_yaml(CONFIG)

    saturated = allocator.allocate(
        [10000.0, 0.0, 0.0, 0.0, 0.0, 0.0], command_limit=0.2
    )
    neutral = allocator.allocate(
        [1.0, 0.0, 0.0, 0.0, 0.0, 0.0], command_limit=0.0
    )

    assert max(abs(value) for value in saturated.commands) <= 0.2 + 1e-9
    assert saturated.saturation_fraction > 0.0
    assert np.allclose(neutral.commands, np.zeros(8))
    assert neutral.saturation_fraction == 1.0


def test_allocator_uses_bounded_scipy_solver_without_legacy_active_set():
    source = (
        Path(__file__).resolve().parents[1]
        / "ros_ws/src/robotcore_control/robotcore_control/thruster_allocation.py"
    ).read_text(encoding="utf-8")

    assert "from scipy.optimize import least_squares, lsq_linear" in source
    assert 'method="trf"' in source
    assert 'method="bvls"' in source
    assert "for _ in range(8)" not in source


def test_vector_model_uses_nonlinear_solver_not_scalar_bvls(monkeypatch):
    allocator = ThrusterAllocator.from_yaml(CONFIG)

    def unexpected_bounded_solve(*_args, **_kwargs):
        raise AssertionError("feasible allocation should not enter BVLS")

    monkeypatch.setattr(allocation_module, "lsq_linear", unexpected_bounded_solve)
    source_commands = [0.12, -0.10, 0.08, -0.06, 0.14, -0.12, 0.10, -0.08]
    target = allocator.wrench_for_commands(source_commands)
    result = allocator.allocate(target)

    assert result.residual < 0.05


def test_pwm_model_clamps_endpoints_and_obeys_deadband():
    allocator = ThrusterAllocator.from_yaml(CONFIG)
    thruster = allocator.thrusters[0]
    assert np.allclose(thruster.force_vector_for_pwm(1500.0), np.zeros(3))
    assert np.allclose(thruster.force_vector_for_pwm(1475.0), np.zeros(3))
    assert np.allclose(thruster.force_vector_for_pwm(1525.0), np.zeros(3))
    assert np.allclose(
        thruster.force_vector_for_pwm(1800.0),
        thruster.force_vector_for_pwm(1700.0),
    )
    assert np.allclose(
        thruster.force_vector_for_pwm(1200.0),
        thruster.force_vector_for_pwm(1300.0),
    )
    assert np.isclose(thruster.command_for_effective(175.0), 0.4)
    assert np.isclose(thruster.command_for_effective(-175.0), -0.4)


def test_vertical_subset_allocator_keeps_horizontal_thrusters_neutral():
    allocator = ThrusterAllocator.from_yaml(CONFIG)

    result = allocator.allocate_subset([0.0, 0.0, 8.0, 0.0, 0.0, 0.0], range(4))
    achieved = allocator.wrench_for_commands(result.commands)

    assert np.allclose(result.commands[4:], np.zeros(4))
    assert achieved[2] > 6.0
    # Installed T1--T4 produce -Z/down force at positive PWM, so +Z requires
    # negative commands.
    assert np.mean(result.commands[:4]) < 0.0


def test_field_verified_positive_pwm_thruster_pair_yaw_directions():
    allocator = ThrusterAllocator.from_yaml(CONFIG)
    positive_pwm = 0.2

    t5_t8 = np.zeros(8)
    t5_t8[[4, 7]] = positive_pwm
    t6_t7 = np.zeros(8)
    t6_t7[[5, 6]] = positive_pwm

    # base_link is FLU: +Tz is counter-clockwise when viewed from above.
    assert allocator.wrench_for_commands(t5_t8)[5] > 0.1
    assert allocator.wrench_for_commands(t6_t7)[5] < -0.1


def test_station_yaw_balance_reduces_translation_in_measured_thruster_model():
    allocator = ThrusterAllocator.from_yaml(CONFIG)

    counter_clockwise = altitude_station_pwm_commands(
        0.0, 0.0, 0.0, 0.08, 0.2, -1.0
    )
    clockwise = altitude_station_pwm_commands(
        0.0, 0.0, 0.0, -0.08, 0.2, -1.0
    )
    ccw_wrench = allocator.wrench_for_commands(counter_clockwise)
    cw_wrench = allocator.wrench_for_commands(clockwise)

    assert ccw_wrench[5] > 0.07
    assert cw_wrench[5] < -0.07
    assert np.linalg.norm(ccw_wrench[:2]) < 0.06
    assert np.linalg.norm(cw_wrench[:2]) < 0.06

    full_ccw = altitude_station_pwm_commands(
        0.0, 0.0, 0.0, 0.20, 0.4, -1.0
    )
    full_wrench = allocator.wrench_for_commands(full_ccw)
    assert full_wrench[5] > 0.45
    assert np.linalg.norm(full_wrench[:2]) < 0.08


def test_station_sway_balance_is_lateral_in_measured_thruster_model():
    allocator = ThrusterAllocator.from_yaml(CONFIG)
    commands = altitude_station_pwm_commands(
        0.0, 0.0, 0.08, 0.0, 0.2, -1.0
    )
    wrench = allocator.wrench_for_commands(commands)

    assert wrench[1] > 0.25
    assert abs(wrench[0]) < 0.03
    assert abs(wrench[5]) < 0.02


def test_t7_t8_reverse_pwm_reverses_individual_yaw_direction():
    allocator = ThrusterAllocator.from_yaml(CONFIG)

    for channel in (6, 7):
        positive = np.zeros(8)
        negative = np.zeros(8)
        positive[channel] = 0.2
        negative[channel] = -0.2

        positive_yaw = allocator.wrench_for_commands(positive)[5]
        negative_yaw = allocator.wrench_for_commands(negative)[5]
        assert abs(positive_yaw) > 0.05
        assert abs(negative_yaw) > 0.05
        assert positive_yaw * negative_yaw < 0.0


def test_field_verified_positive_pwm_vertical_collective_moves_down():
    allocator = ThrusterAllocator.from_yaml(CONFIG)
    commands = np.zeros(8)
    commands[:4] = 0.2

    assert allocator.wrench_for_commands(commands)[2] < -6.0


def test_fast_station_level_mixer_has_correct_measured_roll_and_pitch_signs():
    allocator = ThrusterAllocator.from_yaml(CONFIG)
    positive_roll = altitude_level_pwm_commands(0.0, 0.12, 0.0, 0.4, -1.0)
    positive_pitch = altitude_level_pwm_commands(0.0, 0.0, 0.12, 0.4, -1.0)

    assert allocator.wrench_for_commands(positive_roll)[3] > 0.5
    assert allocator.wrench_for_commands(positive_pitch)[4] > 0.35
    assert max(abs(value) for value in positive_roll) <= 0.4
    assert max(abs(value) for value in positive_pitch) <= 0.4


def test_vector_model_positions_are_already_relative_to_center_of_mass():
    allocator = ThrusterAllocator.from_yaml(CONFIG)

    assert np.allclose(allocator.thrusters[0].position_m, [0.134, -0.160, -0.17098])
    assert np.allclose(allocator.thrusters[3].position_m, [-0.150, 0.160, -0.17098])
