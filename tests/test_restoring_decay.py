import importlib.util
import math
import sys
from pathlib import Path

import numpy as np
import pytest


SCRIPT = Path(__file__).resolve().parents[1] / "scripts/robotcore_restoring_decay_test.py"
SPEC = importlib.util.spec_from_file_location("restoring_decay", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def test_free_decay_fit_recovers_normalized_restoring_and_damping():
    dt = 0.01
    time_s = np.arange(0.0, 18.0, dt)
    angle = np.zeros(time_s.size)
    rate = np.zeros(time_s.size)
    angle[0] = math.radians(75.0)
    linear_damping = 0.42
    quadratic_damping = 0.06
    restoring = 2.25
    for index in range(time_s.size - 1):
        acceleration = (
            -linear_damping * rate[index]
            - quadratic_damping * abs(rate[index]) * rate[index]
            - restoring * math.sin(angle[index])
        )
        rate[index + 1] = rate[index] + acceleration * dt
        angle[index + 1] = angle[index] + rate[index + 1] * dt

    result = MODULE.fit_decay(time_s, angle, rate, effective_inertia_kg_m2=0.8)

    assert result["fit_r_squared"] > 0.98
    assert result["linear_damping_over_inertia_1_s"] == pytest.approx(
        linear_damping, abs=0.04
    )
    assert result["restoring_over_inertia_rad_s2"] == pytest.approx(restoring, abs=0.05)
    assert result["restoring_torque_at_90_deg_n_m"] == pytest.approx(1.8, abs=0.05)
