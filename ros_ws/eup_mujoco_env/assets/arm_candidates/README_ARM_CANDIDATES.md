# Arm Candidate Notes

## Status

The real underwater manipulator is not selected yet. Phase 1/2 uses:

- `eup_mujoco_env/models/generic_6dof_arm.xml`
- `eup_mujoco_env/models/generic_7dof_arm.xml` as a reserved extension point

## Selection Questions

- Exact arm model and vendor.
- Joint count and joint limits.
- URDF, MJCF, CAD, or mesh availability.
- Driver protocol.
- Whether policy output should be joint target, joint delta, or end-effector
  delta.
- Payload and underwater operating assumptions.

## Replacement Contract

The selected arm must continue to publish:

- `/robot/arm_state`

And consume:

- `/control/arm_cmd`

Policy and UI code must not import arm-driver internals.
