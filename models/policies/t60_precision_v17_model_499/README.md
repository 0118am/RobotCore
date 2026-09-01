# t60_precision_v17 model_499

This package contains
`auv_traj_policy_v17_mlp_history_8_2026-08-31_model_499.onnx`, deployed
verbatim as `policy.onnx`. Its SHA-256 is
`b2150b223da1f92bde506d35d23d6a73551a4a1c0fd2976185d7393d4208e2a1`.
The deployable contract is recorded in `policy.yaml` and implemented by
`T60ObservationState`.

The model uses observation contract `t60_trajectory_obs_v11`: one 30-value
current observation followed by 8 newest-first 21-value history samples, for
198 actor inputs spanning 320 ms of past state. The current frame contains
position error, target linear velocity, linear-velocity error, attitude error,
measured and target angular velocity, target linear acceleration, and the
previous motor command. Each history frame contains position error,
linear-velocity error, attitude error, angular-velocity error defined as
`measured - target`, and the previous motor command. Inputs are divided by the
fixed physical scales recorded in the manifest; they are not raw SI values and
do not use running normalization.

The ONNX tensors are `obs[1,198]` and `actions[1,8]`. The model runs at 25 Hz
and emits direct tanh-bounded T1..T8 actions in `[-1, 1]`.
The training critic consumes the same 198 values plus 60 privileged values
(258 total); the deployable ONNX contains only the actor.

Use `t60_policy_shadow.launch.py` for inference-only validation. That launch
publishes only `/policy/body/action`; it does not request command authority,
arm the vehicle, or write a hardware command topic.

The model was trained against the physical vehicle's T1..T8 channel order and
polarity. Each output is copied directly to the same action index, with no
permutation or sign change. The hardware boundary performs the physical
conversion `PWM_us = 1500 + 250 * action`, so `[-1, 1]` corresponds exactly to
`[1250, 1750]` us. Before that conversion, `command_authority` multiplies all
eight actions by the fixed live ratio `pwm_limit_us / 250`; it never clips
channels independently.

The command authority publishes hardware frames at 50 Hz. Each 25 Hz policy
action is consequently held for two consecutive PWM frames.

The policy consumes pose and linear velocity from `/robot/body_state`, angular
velocity from the base-link `/sensors/external_imu`, trajectory state from
`/runtime/trajectory_target`, and the previously accepted actuator command from
`/control/thruster_cmd`. That live canonical command enters the next policy
observation unchanged. The 50 ms state delay is applied to both BodyState and
external IMU samples. History resets whenever inputs become invalid or the
policy is reloaded.
