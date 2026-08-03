# Isaac WarpAUV Trajectory Body Policy (offline only)

This policy was exported from IsaacLab RSL-RL:

```text
/home/jining_yang/IsaacLab/logs/rsl_rl/warpauv_traj_direct/2026-06-12_14-33-57/exported/policy.onnx
```

The source task is `Isaac-WarpAUV-Traj-Direct-v1`.

This artifact is deliberately quarantined from RobotCore deployment: it emits
six actions for a deleted simulator model, while the physical vehicle contract
is eight thrusters. Its manifest is named `offline_policy.yaml`, so normal
policy selection cannot load it as a deployable package.

Historical contract:

- Observation: 20-D IsaacLab trajectory observation.
- Action: six WarpAUV actuators; no mapping to the physical eight-thruster
  vehicle has been validated.

The ONNX/PT files remain only for reproducibility and offline inspection.
