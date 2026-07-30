# Isaac AUV -> RobotCore MuJoCo Migration Checklist

This checklist is the authoritative migration plan for moving useful pieces from
`/home/jining_yang/isaac-auv-env` into EUPSystemInfraPack.

## References Checked

- MuJoCo official docs: `mjData.xfrc_applied` is the Cartesian body wrench input
  used for externally applied forces and torques.
- MuJoCo official XML reference: site actuators with `gear` are the documented
  pattern for jets and propellers.
- MuJoCo Python bindings: `mj_step(model, data)` is the normal simulation step
  entry point used by the Python bridge.
- Isaac AUV local license: BSD-3-Clause. Migrated source-derived code must keep
  attribution and license notes.

## Migration Items

| Priority | Source | Destination | Status | Verification |
| --- | --- | --- | --- | --- |
| P0 | `warpauv_env.py` trajectory generator | `src/eup_runtime/eup_runtime/trajectory_command_node.py` | Start now | Publishes `/runtime/trajectory_target`; launch exposes trajectory args |
| P0 | Isaac 20-D observation contract | `src/eup_policy/eup_policy/runners/onnx_runner.py` and `BodyPolicyNode` | Start now | ONNX runner uses real target fields before manifest defaults |
| P0 | `rigid_body_hydrodynamics.py` math | `eup_mujoco_env/eup_mujoco_env/hydrodynamics.py` | Start now | Pure-Python tests for damping, buoyancy, added-mass power |
| P0 | `thruster_dynamics.py` PWM/thrust model | `src/eup_control/eup_control/warpauv_thruster_model.py` | Start now | Tests cover deadband, polynomial mapping, first-order lag |
| P1 | WarpAUV 6-thruster layout | `eup_mujoco_env/models/warpauv_6thruster.xml` | Start now | MuJoCo launch can select the 6-thruster model |
| P1 | WarpAUV physical constants | `eup_mujoco_env/config/warpauv_dynamics.yaml` | Start now | Config records mass, volume, damping, current, randomization range |
| P2 | Isaac eval scripts | `scripts/eval_mujoco_policy.py` | Later | ROS2 replay/eval logs RMSE, action energy, safety events |
| P2 | Old `.pt` weights | `models/policies/` | Later | Export to ONNX/PT manifest after runner path is stable |

## Boundary Decisions

- Do not migrate IsaacLab runtime classes such as `DirectRLEnv`, `RigidObject`,
  `AppLauncher`, or USD debug drawing. They belong to Isaac/PhysX, not the
  ROS2/MuJoCo runtime.
- Keep Isaac-specific math as reusable pure Python where possible so desktop,
  Jetson, and CI tests do not need IsaacLab installed.
- Keep the 8-thruster BlueROV2 skeleton for the hardware-facing target, but add
  a separate 6-thruster WarpAUV model for validating the exported Isaac policy
  without actuator-count mismatch.
- The trajectory policy is only meaningful when the target fields in its 20-D
  observation are generated from the same trajectory family used during Isaac
  training and evaluation.

## Current Execution Scope

This pass migrates the P0/P1 foundations:

1. Add the `TrajectoryTarget` ROS interface.
2. Add a ROS2 trajectory command node with Isaac-compatible trajectory types.
3. Update BodyPolicy to consume trajectory target status when the manifest asks
   for it.
4. Update ONNX observation construction to prefer real target data.
5. Add WarpAUV thruster and hydrodynamic math modules with focused tests.
6. Add a selectable WarpAUV 6-thruster MJCF and launch arguments.
