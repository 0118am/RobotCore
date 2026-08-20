from pathlib import Path
import sys

import numpy as np


PACKAGE_ROOT = Path(__file__).resolve().parents[1] / "ros_ws" / "src" / "robotcore_control"
sys.path.insert(0, str(PACKAGE_ROOT))

import robotcore_control.thruster_allocation as allocation_module  # noqa: E402
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
    assert np.mean(result.commands[:4]) > 0.0


def test_vector_model_positions_are_already_relative_to_center_of_mass():
    allocator = ThrusterAllocator.from_yaml(CONFIG)

    assert np.allclose(allocator.thrusters[0].position_m, [0.134, -0.160, -0.17098])
    assert np.allclose(allocator.thrusters[3].position_m, [-0.150, 0.160, -0.17098])
