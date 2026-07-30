# Model Selection Notes

Phase 0 owns model-source confirmation. Until real assets are selected, the
repository provides equivalent simplified models that preserve the interface and
control topology.

## BlueROV2 Heavy 8-Thruster Target

Target:

- Reference vehicle: BlueROV2 Heavy-style 8-thruster configuration.
- Purpose: validate 6DoF body policy, thruster allocation, safety, and logging.
- Phase 1/2 fallback: simplified equivalent MJCF with 8 actuator sites.

Required asset note:

- `eup_mujoco_env/assets/bluerov2_heavy_8thruster/README_MODEL.md`

Selection tasks:

- [ ] Identify source asset: URDF, Gazebo model, CAD, mesh, or reconstructed
      MJCF.
- [ ] Confirm source license and redistribution rules.
- [ ] Document conversion pipeline and known issues.
- [ ] Confirm thruster layout, signs, frames, and numbering.
- [ ] Confirm mass, inertia, buoyancy, drag, and hydrodynamics assumptions.

## Arm Target

Target:

- Phase 1/2 fallback: `generic_6dof_arm.xml`.
- Optional placeholder: `generic_7dof_arm.xml`.
- Final target: selected real underwater manipulator after Phase 0.

Required asset note:

- `eup_mujoco_env/assets/arm_candidates/README_ARM_CANDIDATES.md`

Selection tasks:

- [ ] Confirm arm model, joint count, limits, and payload.
- [ ] Confirm URDF/MJCF/CAD availability.
- [ ] Confirm driver protocol and ROS 2 integration path.
- [ ] Confirm joint command type: joint target, joint delta, or EE delta.

## MuJoCo Version

The Python package `mujoco` must be pinned after smoke testing on the target
development and deployment machines. Record the selected version here:

```text
mujoco==TBD
```
