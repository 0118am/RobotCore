"""Validated 8-thruster geometry, bounded allocation, and curve inversion."""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
from pathlib import Path
from typing import Iterable

import numpy as np
import yaml

from .control_math import vec


@dataclass(frozen=True)
class AllocationResult:
    commands: tuple[float, ...]
    forces_n: tuple[float, ...]
    residual: float
    saturation_fraction: float


@dataclass(frozen=True)
class Thruster:
    channel: int
    name: str
    position_m: np.ndarray
    direction: np.ndarray
    command_sign: float
    command_curve: np.ndarray
    thrust_curve_n: np.ndarray

    @property
    def minimum_force(self) -> float:
        return float(self.thrust_curve_n[0])

    @property
    def maximum_force(self) -> float:
        return float(self.thrust_curve_n[-1])

    def force_to_command(self, force_n: float) -> float:
        bounded = float(np.clip(force_n, self.minimum_force, self.maximum_force))
        native = float(np.interp(bounded, self.thrust_curve_n, self.command_curve))
        return float(np.clip(self.command_sign * native, -1.0, 1.0))


class ThrusterAllocator:
    """Map body wrench ``[Fx,Fy,Fz,Tx,Ty,Tz]`` to eight bounded commands."""

    def __init__(
        self,
        thrusters: Iterable[Thruster],
        *,
        damping: float = 1e-4,
        axis_weights: Iterable[float] = (1.0, 1.0, 1.0, 1.0, 1.0, 1.0),
        measured: bool = False,
        config_hash: str = "",
    ):
        self.thrusters = sorted(list(thrusters), key=lambda item: item.channel)
        if len(self.thrusters) != 8 or [item.channel for item in self.thrusters] != list(range(8)):
            raise ValueError("exactly eight unique channels numbered 0..7 are required")
        self.measured = bool(measured)
        self.config_hash = str(config_hash)
        self.damping = max(0.0, float(damping))
        self.axis_weights = vec(axis_weights, 6)
        if np.any(self.axis_weights <= 0.0):
            raise ValueError("axis weights must be positive")

        columns = []
        for item in self.thrusters:
            direction_norm = float(np.linalg.norm(item.direction))
            if not np.isclose(direction_norm, 1.0, atol=1e-4):
                raise ValueError(f"thruster {item.name} direction must be a unit vector")
            columns.append(np.concatenate([item.direction, np.cross(item.position_m, item.direction)]))
        self.matrix = np.asarray(columns, dtype=np.float64).T
        self.rank = int(np.linalg.matrix_rank(self.matrix))
        self.condition = float(np.linalg.cond(self.matrix))
        if self.rank != 6:
            raise ValueError(f"thruster allocation matrix rank is {self.rank}, expected 6")
        if not np.isfinite(self.condition):
            raise ValueError("thruster allocation matrix condition is not finite")

    @classmethod
    def from_yaml(cls, path: str | Path) -> "ThrusterAllocator":
        config_path = Path(path)
        data = yaml.safe_load(config_path.read_text(encoding="utf-8")) or {}
        canonical = json.dumps(data, sort_keys=True, separators=(",", ":"))
        config_hash = hashlib.sha256(canonical.encode("utf-8")).hexdigest()[:16]
        entries = data.get("thrusters") or []
        thrusters = []
        for entry in entries:
            curve = np.asarray(entry.get("curve") or [], dtype=np.float64)
            if curve.shape[0] < 3 or curve.shape[1:] != (2,):
                raise ValueError(f"thruster {entry.get('name', '?')} requires at least three curve points")
            order = np.argsort(curve[:, 1])
            curve = curve[order]
            if np.any(np.diff(curve[:, 1]) <= 0.0):
                raise ValueError(f"thruster {entry.get('name', '?')} thrust curve must be strictly monotonic")
            if np.any(np.diff(curve[:, 0]) <= 0.0):
                raise ValueError(f"thruster {entry.get('name', '?')} command curve must be strictly monotonic")
            if curve[0, 0] < -1.0 or curve[-1, 0] > 1.0:
                raise ValueError("curve commands must remain in [-1, 1]")
            thrusters.append(
                Thruster(
                    channel=int(entry["channel"]),
                    name=str(entry["name"]),
                    position_m=vec(entry["position_m"], 3),
                    direction=vec(entry["direction"], 3),
                    command_sign=float(entry.get("command_sign", 1.0)),
                    command_curve=curve[:, 0],
                    thrust_curve_n=curve[:, 1],
                )
            )
        return cls(
            thrusters,
            damping=float(data.get("damping", 1e-4)),
            axis_weights=data.get("axis_weights", [1.0] * 6),
            measured=bool(data.get("measured", False)),
            config_hash=config_hash,
        )

    def allocate(self, wrench: Iterable[float]) -> AllocationResult:
        target = vec(wrench, 6)
        weights = np.diag(self.axis_weights)
        weighted_matrix = weights @ self.matrix
        weighted_target = weights @ target
        lower = np.asarray([item.minimum_force for item in self.thrusters])
        upper = np.asarray([item.maximum_force for item in self.thrusters])
        forces = np.zeros(8, dtype=np.float64)
        free = list(range(8))
        fixed: list[int] = []

        for _ in range(8):
            residual_target = weighted_target.copy()
            if fixed:
                residual_target -= weighted_matrix[:, fixed] @ forces[fixed]
            if free:
                active = weighted_matrix[:, free]
                regularized = active @ active.T + self.damping * np.eye(6)
                solved = active.T @ np.linalg.solve(regularized, residual_target)
                forces[free] = solved

            violations = [
                index
                for index in free
                if forces[index] < lower[index] or forces[index] > upper[index]
            ]
            if not violations:
                break
            # Clamp the worst normalized violation first, then recompute the
            # remaining free thrusters against the residual wrench.
            def violation_size(index):
                span = max(upper[index] - lower[index], 1e-9)
                return max(lower[index] - forces[index], forces[index] - upper[index], 0.0) / span

            index = max(violations, key=violation_size)
            forces[index] = float(np.clip(forces[index], lower[index], upper[index]))
            free.remove(index)
            fixed.append(index)

        forces = np.clip(forces, lower, upper)
        achieved = self.matrix @ forces
        residual = float(np.linalg.norm(self.axis_weights * (target - achieved)))
        at_limit = np.logical_or(
            np.isclose(forces, lower, atol=1e-6),
            np.isclose(forces, upper, atol=1e-6),
        )
        commands = tuple(
            item.force_to_command(float(forces[index]))
            for index, item in enumerate(self.thrusters)
        )
        return AllocationResult(
            commands=commands,
            forces_n=tuple(float(value) for value in forces),
            residual=residual,
            saturation_fraction=float(np.mean(at_limit)),
        )

