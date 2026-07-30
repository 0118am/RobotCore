# Isaac WarpAUV Trajectory Body Policy

This policy was exported from IsaacLab RSL-RL:

```text
/home/jining_yang/IsaacLab/logs/rsl_rl/warpauv_traj_direct/2026-06-12_14-33-57/exported/policy.onnx
```

The source task is `Isaac-WarpAUV-Traj-Direct-v1`.

Important contract:

- Observation: 20-D IsaacLab trajectory observation.
- Action: 6-D normalized WarpAUV thruster/PWM command.
- EUP now includes `eup_mujoco_env/models/warpauv_6thruster.xml` for this
  6-output policy. The older BlueROV2-style 8-thruster skeleton can still run
  the ROS graph, but it is not the recommended dynamics match.

Trajectory target fields are provided by `/runtime/trajectory_target`, published
by `trajectory_command_node`. Manifest defaults remain only as an ONNX smoke-test
fallback.
