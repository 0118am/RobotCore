from pathlib import Path
import sys

import numpy as np


PACKAGE_ROOT = Path(__file__).resolve().parents[1] / "ros_ws" / "src" / "eup_control"
sys.path.insert(0, str(PACKAGE_ROOT))

from eup_control.thruster_allocation import ThrusterAllocator  # noqa: E402


CONFIG = (
    Path(__file__).resolve().parents[1]
    / "ros_ws"
    / "src"
    / "eup_control"
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


def test_forward_and_reverse_curve_inversion_is_asymmetric():
    allocator = ThrusterAllocator.from_yaml(CONFIG)
    thruster = allocator.thrusters[0]
    assert np.isclose(thruster.force_to_command(25.0), 0.5)
    assert np.isclose(thruster.force_to_command(-20.0), -0.5)

