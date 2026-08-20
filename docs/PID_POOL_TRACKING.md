# Six-DoF PID Pool Tracking

This control path is designed for a tethered, eight-thruster vehicle. It is
fail-closed by default: the checked-in PID gains, thruster geometry, and pool
bounds are deliberately marked unconfirmed, so PID cannot arm after a fresh
checkout.

## Required measurements

Edit the installed-source equivalents of:

- `ros_ws/src/robotcore_control/config/real_pool_thrusters.yaml`
- `/var/lib/robotcore/config/pid/active.json`
- `ros_ws/src/robotcore_control/config/real_pool_safety.yaml`

For every thruster record the physical channel, `base_link` position, positive
force direction, wiring/ESC sign, and a monotonic command-to-thrust curve.
Replace all placeholder values. Set `measured: true` only after a guarded
channel/sign test. The allocator rejects non-unit directions, duplicate/missing
channels, non-monotonic curves, and geometry whose wrench matrix rank is not
six.

The browser's collapsed **PID Tuning** panel writes a named profile to
`/var/lib/robotcore/config/pid/profiles/<profile>.json` and the same complete
document to `pid/active.json`. Set `configured: true` only after the staged
single-axis tuning has been reviewed. The next **Arm** reloads the entire active
document before authority can arm; no controller process restart is needed.
The browser then reads `/control/pid/config`, so its `LIVE` values and hash are
the exact configuration held by the running PID process.

Put the measured map-frame pool envelope into both node
sections of `real_pool_safety.yaml`, set the approved linear/angular speed and
absolute RPY ranges, then set `pool_bounds_configured: true` and
`trajectory_limits_configured: true`. A scenario is rejected unless its full
position, attitude, and speed envelope stays within those values.

## Runtime authority

Only `command_authority` publishes `/control/thruster_cmd`. Manual, PID, and
future RL sources publish isolated candidates. A source can change only while
disarmed. PID arming also requires:

- a current healthy safety heartbeat;
- a fresh enabled PID candidate;
- one trusted absolute AprilTag alignment since startup;
- a fresh valid body state and valid linear velocity;
- a fresh valid trajectory target inside the configured pool envelope.

Any candidate, body-state, target, or safety timeout disarms and latches a
fault. Clear the underlying abort first, then clear the authority fault and
explicitly arm again.

```bash
ros2 service call /control/authority/set \
  robotcore_interfaces/srv/SetControlAuthority \
  "{source: pid, arm: false, clear_fault: true}"

ros2 service call /control/authority/set \
  robotcore_interfaces/srv/SetControlAuthority \
  "{source: pid, arm: true, clear_fault: false}"
```

The edge launch starts the complete path:

```bash
cd /home/nvidia/RobotCore/ros_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch robotcore_bringup robotcore_edge_system.launch.py enable_web_ui:=false
```

The browser exposes the same source selection, preflight details, Arm, Disarm,
fault clear, and Abort actions. Manual sliders publish only
`/control/candidates/manual`.

## Controlled experiments

Use this order in the pool. Do not start with the six-axis trajectory.

1. Verify thruster channel/sign and allocation while restrained and disarmed.
2. Run `pose_hold`; its initial map-frame target height is `Z=0.9 m`.
   Right-stick up/down commands vertical speed without the
   old discount coefficient. Releasing the stick immediately requests zero
   speed and latches measured map-frame height, so no accumulated target
   remains. The height controller is a separate actuator-domain PID; it does
   not invert a requested force through the measured thrust curves. Positive
   FLU vertical effort is converted to the verified negative installed PWM
   sign and applied equally to T1–T4, bounded only by the unified PWM Limit.
   A low-pass filter is applied to measured vertical velocity, not to the PWM
   command. Forward/back plus yaw on T5–T8 remain under left-stick control.
   During the short Select+Arm hand-off before explicit Start, the PID
   candidate remains enabled but all eight commands are exactly neutral.
3. Run `step_x`, `step_y`, and `step_z` separately. Increase `Kp` to obtain a
   clear response, add `Kd` to control overshoot, then only enough `Ki` to
   remove steady bias.
4. Repeat with `step_roll`, `step_pitch`, and `step_yaw`.
5. Tune the outer position/orientation proportional gains and velocity limits.
6. Only after all six step tests pass, run `pose_lissajous_6dof` as the coupled
   validation experiment.

Change one gain family at a time and save a new profile name for every run.
This makes the run's configuration snapshot an unambiguous experimental
record rather than relying on operator notes.

Experiments never arm the vehicle. Select PID and arm it first, then request an
allow-listed scenario:

```bash
ros2 action send_goal /runtime/run_tracking_experiment \
  robotcore_interfaces/action/RunTrackingExperiment \
  "{scenario: pose_lissajous_6dof, controller: pid, duration_s: 70.0}" \
  --feedback
```

The same flow is available in the browser: choose a managed task, click Arm,
then Start. The action resets the task clock, runs the configured hold/tracking
phases, and always disarms when it completes, is canceled, or detects an
authority fault. Task definitions are individual JSON files under
`/var/lib/robotcore/config/tasks/`; each goal reloads its file, so editing a task
does not require restarting the action server.

Starting an accepted experiment creates exactly one directory:

```text
/var/lib/robotcore/runs/run_<timestamp>_<task>/
  configs/                 exact PID, task, thruster and safety snapshots
  rosbag2/tracking/        full-rate rosbag2 SQLite recording
  event_log.jsonl          bounded, operator-readable summaries and markers
  rosbag2.log              recorder output
```

The topic allow-list is `/var/lib/robotcore/config/tasks/record_topics.json`.
The bag is the primary evidence because it preserves source messages and ROS
timestamps. JSONL is a convenient index and quick-analysis input, not a
replacement for the bag.

Analyze one or more completed run directories with:

```bash
python3 scripts/analyze_tracking_runs.py \
  /var/lib/robotcore/runs/run_YYYYMMDD_HHMMSS_task \
  --output tracking_report
```

The report contains CSV/JSON metrics, trajectory delay, step settling time,
six-axis pose/velocity/error figures, thruster figures, and a multi-run summary
PNG. A run passes only when position RMSE is at most 0.15 m, orientation
geodesic RMSE is at most 10 degrees, saturation is at most 5%, valid samples
are at least 99%, and no abort or authority fault occurred.

## RL boundary

`/control/candidates/rl` is reserved for a future eight-output policy adapter.
The adapter requires exactly eight finite outputs, a measured thruster
configuration, and a non-empty `policy_layout_hash` equal to that
configuration's hash. The edge authority rejects RL by default. The retained
six-thruster WarpAUV weights are offline provenance only: they have no
deployable `policy.yaml` and must not be mapped or padded onto this vehicle.
