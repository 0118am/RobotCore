"""Pure-Python Fossen-style hydrodynamic wrench helpers.

This module is derived from `/home/jining_yang/isaac-auv-env/rigid_body_hydrodynamics.py`
and keeps the same boundary: MuJoCo integrates rigid-body inertia, gravity, and
contacts, while this code returns only the external fluid wrench to apply to the
vehicle body.  The source project is BSD-3-Clause licensed.
"""

from __future__ import annotations

from dataclasses import dataclass, field


Vector3 = tuple[float, float, float]
Vector6 = tuple[float, float, float, float, float, float]


def _vec3(values) -> Vector3:
    return (float(values[0]), float(values[1]), float(values[2]))


def add3(a: Vector3, b: Vector3) -> Vector3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def sub3(a: Vector3, b: Vector3) -> Vector3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def cross(a: Vector3, b: Vector3) -> Vector3:
    """Return a x b."""

    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def quat_conjugate_wxyz(quat) -> tuple[float, float, float, float]:
    """Quaternion conjugate for IsaacLab and MuJoCo wxyz ordering."""

    return (float(quat[0]), -float(quat[1]), -float(quat[2]), -float(quat[3]))


def quat_apply_wxyz(quat, vector) -> Vector3:
    """Rotate a vector using a wxyz quaternion."""

    w, x, y, z = (float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3]))
    v = _vec3(vector)
    xyz = (x, y, z)
    t = tuple(2.0 * item for item in cross(xyz, v))
    return add3(add3(v, tuple(w * item for item in t)), cross(xyz, t))


@dataclass(frozen=True)
class HydrodynamicConfig:
    """Physical constants for one rigid AUV body."""

    fluid_density: float = 997.0
    volume_m3: float = 0.022747843530591776
    com_to_cob_m: Vector3 = (0.0, 0.0, 0.01)
    linear_damping: Vector6 = (0.00526, 0.00526, 0.00526, 0.00032, 0.00032, 0.00032)
    quadratic_damping: Vector6 = (39.196, 68.272, 135.402, 0.277, 1.387, 0.770)
    added_mass_diag: Vector6 = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    water_current_w: Vector3 = (0.0, 0.0, 0.0)
    gravity_w: Vector3 = (0.0, 0.0, -9.81)


@dataclass
class HydrodynamicForceModel:
    """Computes body-frame fluid force and torque."""

    config: HydrodynamicConfig = field(default_factory=HydrodynamicConfig)

    def calculate_buoyancy_forces(self, root_quat_w) -> tuple[Vector3, Vector3]:
        """Return buoyancy force and torque in body coordinates."""

        cfg = self.config
        buoyancy_force_w = tuple(-cfg.fluid_density * cfg.volume_m3 * item for item in cfg.gravity_w)
        buoyancy_force_b = quat_apply_wxyz(quat_conjugate_wxyz(root_quat_w), buoyancy_force_w)
        buoyancy_torque_b = cross(cfg.com_to_cob_m, buoyancy_force_b)
        return buoyancy_force_b, buoyancy_torque_b

    def calculate_fluid_wrench(
        self,
        *,
        root_quat_w,
        root_linvel_b,
        root_angvel_b,
    ) -> tuple[Vector3, Vector3]:
        """Return body-frame force and torque from buoyancy and damping."""

        cfg = self.config
        buoyancy_force_b, buoyancy_torque_b = self.calculate_buoyancy_forces(root_quat_w)
        water_current_b = quat_apply_wxyz(quat_conjugate_wxyz(root_quat_w), cfg.water_current_w)

        nu_r = (
            float(root_linvel_b[0]) - water_current_b[0],
            float(root_linvel_b[1]) - water_current_b[1],
            float(root_linvel_b[2]) - water_current_b[2],
            float(root_angvel_b[0]),
            float(root_angvel_b[1]),
            float(root_angvel_b[2]),
        )
        damping = self.calculate_relative_damping_wrench(nu_r)
        added_coriolis = self.calculate_added_mass_coriolis_wrench(nu_r)

        force_b = (
            buoyancy_force_b[0] + damping[0] - added_coriolis[0],
            buoyancy_force_b[1] + damping[1] - added_coriolis[1],
            buoyancy_force_b[2] + damping[2] - added_coriolis[2],
        )
        torque_b = (
            buoyancy_torque_b[0] + damping[3] - added_coriolis[3],
            buoyancy_torque_b[1] + damping[4] - added_coriolis[4],
            buoyancy_torque_b[2] + damping[5] - added_coriolis[5],
        )
        return force_b, torque_b

    def calculate_relative_damping_wrench(self, nu_r: Vector6) -> Vector6:
        """Damping wrench; nu_r dot damping is always non-positive."""

        cfg = self.config
        return tuple(
            -(cfg.linear_damping[index] + cfg.quadratic_damping[index] * abs(float(nu_r[index])))
            * float(nu_r[index])
            for index in range(6)
        )

    def calculate_added_mass_coriolis_wrench(self, nu_r: Vector6) -> Vector6:
        """Return C_A(nu_r) nu_r for a diagonal added-mass matrix."""

        cfg = self.config
        if not any(abs(value) > 0.0 for value in cfg.added_mass_diag):
            return (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

        v = _vec3(nu_r[0:3])
        omega = _vec3(nu_r[3:6])
        a_linear = tuple(cfg.added_mass_diag[index] * v[index] for index in range(3))
        a_angular = tuple(cfg.added_mass_diag[index + 3] * omega[index] for index in range(3))
        top = tuple(-item for item in cross(a_linear, omega))
        bottom_cross = add3(cross(a_linear, v), cross(a_angular, omega))
        bottom = tuple(-item for item in bottom_cross)
        return top + bottom
