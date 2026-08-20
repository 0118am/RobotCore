"""Validated 8-thruster geometry, bounded allocation, and curve inversion."""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
from typing import Iterable

import numpy as np
from scipy.optimize import least_squares, lsq_linear
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


@dataclass(frozen=True)
class VectorThruster:
    """Measured PWM-to-vector-force model for one physical thruster."""

    channel: int
    name: str
    position_m: np.ndarray
    neutral_pwm_us: float
    minimum_pwm_us: float
    maximum_pwm_us: float
    deadband_us: float
    hardware_span_us: float
    reverse_quadratic: np.ndarray
    reverse_linear: np.ndarray
    forward_quadratic: np.ndarray
    forward_linear: np.ndarray

    @property
    def reverse_effective_limit_us(self) -> float:
        return self.neutral_pwm_us - self.minimum_pwm_us - self.deadband_us

    @property
    def forward_effective_limit_us(self) -> float:
        return self.maximum_pwm_us - self.neutral_pwm_us - self.deadband_us

    def force_vector_for_effective(self, effective_us: float) -> np.ndarray:
        value = float(np.clip(
            effective_us,
            -self.reverse_effective_limit_us,
            self.forward_effective_limit_us,
        ))
        if value < 0.0:
            scale = -value
            return self.reverse_quadratic * scale * scale + self.reverse_linear * scale
        if value > 0.0:
            return self.forward_quadratic * value * value + self.forward_linear * value
        return np.zeros(3, dtype=np.float64)

    def force_vector_for_pwm(self, pwm_us: float) -> np.ndarray:
        bounded = float(np.clip(pwm_us, self.minimum_pwm_us, self.maximum_pwm_us))
        offset = bounded - self.neutral_pwm_us
        if abs(offset) <= self.deadband_us:
            return np.zeros(3, dtype=np.float64)
        effective = math.copysign(abs(offset) - self.deadband_us, offset)
        return self.force_vector_for_effective(effective)

    def command_for_effective(self, effective_us: float) -> float:
        value = float(np.clip(
            effective_us,
            -self.reverse_effective_limit_us,
            self.forward_effective_limit_us,
        ))
        if math.isclose(value, 0.0, abs_tol=1e-9):
            return 0.0
        pwm_offset = math.copysign(self.deadband_us + abs(value), value)
        return float(np.clip(pwm_offset / self.hardware_span_us, -1.0, 1.0))

    def wrench_for_effective(self, effective_us: float) -> np.ndarray:
        force = self.force_vector_for_effective(effective_us)
        return np.concatenate([force, np.cross(self.position_m, force)])

    def wrench_derivative_for_effective(self, effective_us: float) -> np.ndarray:
        value = float(effective_us)
        if value < 0.0:
            scale = -value
            force_derivative = -(2.0 * self.reverse_quadratic * scale + self.reverse_linear)
        elif value > 0.0:
            force_derivative = 2.0 * self.forward_quadratic * value + self.forward_linear
        else:
            # The force is continuous but its branch slope is not. The mean
            # one-sided derivative lets the optimizer leave neutral without
            # favoring forward or reverse a priori.
            force_derivative = 0.5 * (self.forward_linear - self.reverse_linear)
        return np.concatenate(
            [force_derivative, np.cross(self.position_m, force_derivative)]
        )


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

        self.vector_mode = all(isinstance(item, VectorThruster) for item in self.thrusters)
        if self.vector_mode:
            columns = [
                0.5 * (
                    item.wrench_for_effective(item.forward_effective_limit_us)
                    - item.wrench_for_effective(-item.reverse_effective_limit_us)
                )
                for item in self.thrusters
            ]
        elif all(isinstance(item, Thruster) for item in self.thrusters):
            columns = []
            for item in self.thrusters:
                direction_norm = float(np.linalg.norm(item.direction))
                if not np.isclose(direction_norm, 1.0, atol=1e-4):
                    raise ValueError(f"thruster {item.name} direction must be a unit vector")
                columns.append(np.concatenate([item.direction, np.cross(item.position_m, item.direction)]))
        else:
            raise ValueError("thruster configuration cannot mix scalar and vector force models")
        self.matrix = np.asarray(columns, dtype=np.float64).T
        self.rank = int(np.linalg.matrix_rank(self.matrix))
        self.condition = float(np.linalg.cond(self.matrix))
        if self.rank != 6:
            raise ValueError(f"thruster allocation matrix rank is {self.rank}, expected 6")
        if not np.isfinite(self.condition):
            raise ValueError("thruster allocation matrix condition is not finite")

        # The geometry and limits never change after configuration is loaded.
        # Precompute the bounded, damped least-squares system so the 60 Hz
        # control loop only updates its six-element target vector.
        self.weighted_matrix = self.axis_weights[:, np.newaxis] * self.matrix
        if self.vector_mode:
            self.lower_bounds = -np.ones(8, dtype=np.float64)
            self.upper_bounds = np.ones(8, dtype=np.float64)
            self._last_vector_solution = np.zeros(8, dtype=np.float64)
        else:
            self.lower_bounds = np.asarray(
                [item.minimum_force for item in self.thrusters], dtype=np.float64
            )
            self.upper_bounds = np.asarray(
                [item.maximum_force for item in self.thrusters], dtype=np.float64
            )
        regularized_normal = (
            self.weighted_matrix @ self.weighted_matrix.T
            + self.damping * np.eye(6)
        )
        self.unconstrained_gain = np.linalg.solve(
            regularized_normal, self.weighted_matrix
        ).T
        if self.damping > 0.0:
            self.solver_matrix = np.vstack(
                [self.weighted_matrix, math.sqrt(self.damping) * np.eye(8)]
            )
            self.regularization_target = np.zeros(8, dtype=np.float64)
        else:
            self.solver_matrix = self.weighted_matrix
            self.regularization_target = np.empty(0, dtype=np.float64)

    @classmethod
    def from_yaml(cls, path: str | Path) -> "ThrusterAllocator":
        config_path = Path(path)
        data = yaml.safe_load(config_path.read_text(encoding="utf-8")) or {}
        canonical = json.dumps(data, sort_keys=True, separators=(",", ":"))
        config_hash = hashlib.sha256(canonical.encode("utf-8")).hexdigest()[:16]
        entries = data.get("thrusters") or []
        pwm = data.get("pwm_force_model") or {}
        thrusters = []
        for entry in entries:
            if "reverse_quadratic" in entry:
                thrusters.append(
                    VectorThruster(
                        channel=int(entry["channel"]),
                        name=str(entry["name"]),
                        # Measured-model positions are already expressed from
                        # the vehicle centre of mass in base_link FLU.
                        position_m=vec(entry["position_m"], 3),
                        neutral_pwm_us=float(pwm["neutral_pwm_us"]),
                        minimum_pwm_us=float(pwm["minimum_pwm_us"]),
                        maximum_pwm_us=float(pwm["maximum_pwm_us"]),
                        deadband_us=float(pwm["deadband_us"]),
                        hardware_span_us=float(pwm["hardware_span_us"]),
                        reverse_quadratic=vec(entry["reverse_quadratic"], 3),
                        reverse_linear=vec(entry["reverse_linear"], 3),
                        forward_quadratic=vec(entry["forward_quadratic"], 3),
                        forward_linear=vec(entry["forward_linear"], 3),
                    )
                )
                continue
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
        if self.vector_mode:
            return self._allocate_vector_model(target)
        weighted_target = self.axis_weights * target
        forces = self.unconstrained_gain @ weighted_target
        if not np.all(
            (forces >= self.lower_bounds) & (forces <= self.upper_bounds)
        ):
            # Saturation is uncommon in tuned operation but it must be solved
            # globally when it occurs. SciPy's bounded-variable least-squares
            # implementation replaces the previous one-way clamp heuristic.
            solver_target = np.concatenate(
                [weighted_target, self.regularization_target]
            )
            solution = lsq_linear(
                self.solver_matrix,
                solver_target,
                bounds=(self.lower_bounds, self.upper_bounds),
                method="bvls",
                tol=1e-8,
                max_iter=16,
            )
            if not solution.success or not np.all(np.isfinite(solution.x)):
                raise RuntimeError(
                    f"bounded thruster allocation failed: {solution.message}"
                )
            forces = solution.x
        achieved = self.matrix @ forces
        residual = float(np.linalg.norm(self.axis_weights * (target - achieved)))
        at_limit = np.logical_or(
            np.isclose(forces, self.lower_bounds, atol=1e-6),
            np.isclose(forces, self.upper_bounds, atol=1e-6),
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

    def wrench_for_commands(self, commands: Iterable[float]) -> np.ndarray:
        """Evaluate the exact configured vector model for normalized hardware commands."""

        if not self.vector_mode:
            raise ValueError("wrench_for_commands requires the vector force model")
        values = vec(commands, 8)
        achieved = np.zeros(6, dtype=np.float64)
        for item, command in zip(self.thrusters, values):
            pwm = item.neutral_pwm_us + float(command) * item.hardware_span_us
            force = item.force_vector_for_pwm(pwm)
            achieved += np.concatenate([force, np.cross(item.position_m, force)])
        return achieved

    def allocate_subset(
        self, wrench: Iterable[float], channels: Iterable[int]
    ) -> AllocationResult:
        """Allocate a wrench while keeping every channel outside ``channels`` neutral."""

        if not self.vector_mode:
            raise ValueError("allocate_subset requires the vector force model")
        active = tuple(sorted({int(channel) for channel in channels}))
        if not active or any(channel < 0 or channel >= 8 for channel in active):
            raise ValueError("active thruster channels must be a non-empty subset of 0..7")
        return self._allocate_vector_model(vec(wrench, 6), active_channels=active)

    def _allocate_vector_model(
        self,
        target: np.ndarray,
        active_channels: tuple[int, ...] | None = None,
    ) -> AllocationResult:
        weighted_target = self.axis_weights * target
        active = tuple(range(8)) if active_channels is None else active_channels
        full_seed = np.clip(self.unconstrained_gain @ weighted_target, -1.0, 1.0)
        if np.linalg.norm(self._last_vector_solution) > 1e-8:
            full_seed = self._last_vector_solution.copy()
        active_index = np.asarray(active, dtype=np.int64)
        seed = full_seed[active_index]

        def achieved_for(values):
            achieved = np.zeros(6, dtype=np.float64)
            for channel, normalized in zip(active, values):
                item = self.thrusters[channel]
                effective = (
                    float(normalized) * item.forward_effective_limit_us
                    if normalized >= 0.0
                    else float(normalized) * item.reverse_effective_limit_us
                )
                achieved += item.wrench_for_effective(effective)
            return achieved

        def objective(values):
            residual = self.axis_weights * (target - achieved_for(values))
            if self.damping <= 0.0:
                return residual
            return np.concatenate([residual, math.sqrt(self.damping) * values])

        def jacobian(values):
            columns = []
            for channel, normalized in zip(active, values):
                item = self.thrusters[channel]
                limit = (
                    item.forward_effective_limit_us
                    if normalized >= 0.0
                    else item.reverse_effective_limit_us
                )
                effective = float(normalized) * limit
                columns.append(
                    -self.axis_weights
                    * item.wrench_derivative_for_effective(effective)
                    * limit
                )
            primary = np.asarray(columns, dtype=np.float64).T
            if self.damping <= 0.0:
                return primary
            return np.vstack(
                [primary, math.sqrt(self.damping) * np.eye(len(active))]
            )

        solution = least_squares(
            objective,
            seed,
            jac=jacobian,
            bounds=(-np.ones(len(active)), np.ones(len(active))),
            method="trf",
            max_nfev=12,
            ftol=1e-6,
            xtol=1e-6,
            gtol=1e-6,
        )
        # An infeasible requested wrench can legitimately reach max_nfev; the
        # bounded finite iterate is still the safest best-effort allocation and
        # its residual is reported to the controller/status path.
        if not np.all(np.isfinite(solution.x)):
            raise RuntimeError(f"nonlinear thruster allocation failed: {solution.message}")
        normalized_effective = np.zeros(8, dtype=np.float64)
        normalized_effective[active_index] = np.clip(solution.x, -1.0, 1.0)
        self._last_vector_solution = normalized_effective
        achieved = achieved_for(normalized_effective[active_index])
        commands = []
        signed_forces = []
        for item, normalized in zip(self.thrusters, normalized_effective):
            effective = (
                float(normalized) * item.forward_effective_limit_us
                if normalized >= 0.0
                else float(normalized) * item.reverse_effective_limit_us
            )
            commands.append(item.command_for_effective(effective))
            force = item.force_vector_for_effective(effective)
            signed_forces.append(math.copysign(float(np.linalg.norm(force)), effective))
        residual = float(np.linalg.norm(self.axis_weights * (target - achieved)))
        return AllocationResult(
            commands=tuple(commands),
            forces_n=tuple(signed_forces),
            residual=residual,
            saturation_fraction=float(
                np.mean(
                    np.isclose(
                        np.abs(normalized_effective[active_index]), 1.0, atol=1e-6
                    )
                )
            ),
        )
