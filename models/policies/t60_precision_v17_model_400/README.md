# t60_precision_v17 model_400

This package contains
`auv_traj_policy_v17_mlp_history_8_2026-09-01_model_400.onnx`, deployed
verbatim as `policy.onnx`. Its SHA-256 is
`1a5d16cd0410ada99f693194b3b6d4637419d5bd9d2fbe068831cbf86b8f9b25`.
The deployable contract is recorded in `policy.yaml` and implemented by
`T60ObservationState`.

The model uses observation contract `t60_trajectory_obs_v11`: one 30-value
current observation followed by 8 newest-first 21-value history samples, for
198 actor inputs. The ONNX tensors are `obs[1,198]` and `actions[1,8]`. The
model runs at 25 Hz and emits direct tanh-bounded T1..T8 actions in `[-1, 1]`.

The model is deployed in the physical vehicle's T1..T8 channel order. Only for
commands whose authority source is `command_authority:rl`, the hardware PWM
boundary sign-inverts T5 and T6; all other channels retain their sign. PID and
manual actions are not adapted. The RL conversion is
`PWM_us = 1500 + 250 * rl_polarity[T] * action`, where `rl_polarity` is
`[1, 1, 1, 1, -1, -1, 1, 1]`.
