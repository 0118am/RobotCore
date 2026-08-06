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


def test_placeholder_geometry_is_full_rank_but_not_measured():
    allocator = ThrusterAllocator.from_yaml(CONFIG)
    assert allocator.rank == 6
    assert np.isfinite(allocator.condition)
    assert allocator.measured is False
    assert len(allocator.config_hash) == 16


def test_allocator_tracks_feasible_wrench_and_returns_eight_commands():
    allocator = ThrusterAllocator.from_yaml(CONFIG)
    result = allocator.allocate([5.0, 2.0, 3.0, 0.2, -0.3, 0.4])
    assert len(result.commands) == 8
    assert all(-1.0 <= value <= 1.0 for value in result.commands)
    assert result.residual < 0.05
    assert result.saturation_fraction == 0.0


def test_allocator_bounds_infeasible_wrench():
    allocator = ThrusterAllocator.from_yaml(CONFIG)
    result = allocator.allocate([10000.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    assert all(-1.0 <= value <= 1.0 for value in result.commands)
    assert result.saturation_fraction > 0.0
    assert result.residual > 1000.0


def test_allocator_uses_bounded_scipy_solver_without_legacy_active_set():
    source = (
        Path(__file__).resolve().parents[1]
        / "ros_ws/src/robotcore_control/robotcore_control/thruster_allocation.py"
    ).read_text(encoding="utf-8")

    assert "from scipy.optimize import lsq_linear" in source
    assert 'method="bvls"' in source
    assert "for _ in range(8)" not in source


def test_feasible_wrench_uses_precomputed_fast_path(monkeypatch):
    allocator = ThrusterAllocator.from_yaml(CONFIG)

    def unexpected_bounded_solve(*_args, **_kwargs):
        raise AssertionError("feasible allocation should not enter BVLS")

    monkeypatch.setattr(allocation_module, "lsq_linear", unexpected_bounded_solve)
    result = allocator.allocate([5.0, 2.0, 3.0, 0.2, -0.3, 0.4])

    assert result.residual < 0.05


def test_forward_and_reverse_curve_inversion_is_asymmetric():
    allocator = ThrusterAllocator.from_yaml(CONFIG)
    thruster = allocator.thrusters[0]
    assert np.isclose(thruster.force_to_command(25.0), 0.5)
    assert np.isclose(thruster.force_to_command(-20.0), -0.5)
