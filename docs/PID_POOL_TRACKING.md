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

Put the planned map-frame pool envelope in the `trajectory_command` section of
`real_pool_safety.yaml`, set the approved linear/angular speed and absolute RPY
ranges, then set `trajectory_limits_configured: true`. A scenario is rejected
unless its full planned position, attitude, and speed envelope stays within
those values. Runtime command authority does not trip on measured vehicle
position crossing this planning envelope.

## Runtime authority

Only `command_authority` publishes `/control/thruster_cmd`. Manual, PID, and
future RL sources publish to `/control/manual/thruster_cmd`,
`/control/pid/thruster_cmd`, and `/control/rl/thruster_cmd`, respectively. A
source can change only while disarmed. PID arming also requires:

- a current healthy safety heartbeat;
- a fresh enabled PID command;
- one trusted absolute AprilTag alignment since startup;
- a fresh valid body state and valid linear velocity;
- a fresh valid trajectory target inside the configured pool envelope.

Any source-command, body-state, target, or safety timeout disarms and latches a
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
`/control/manual/thruster_cmd`; the authority subscribes each source's dedicated
topic and rejects a command whose producer identity does not match that topic.

## 定高与定点模式

1. Verify thruster channel/sign and allocation while restrained and disarmed.
2. Run `pose_hold` (shown as `定高模式`); its initial
   map-frame target height is `Z=0.9 m`.
   Right-stick up/down commands vertical speed without the
   old discount coefficient. Releasing the stick immediately requests zero
   speed and latches measured map-frame height, so no accumulated target
   remains. The height controller is a separate actuator-domain PID; it does
   not invert a requested force through the measured thrust curves. Positive
   FLU vertical effort is converted to the verified negative installed PWM
   sign and applied equally to T1–T4, bounded only by the unified PWM Limit.
   A low-pass filter is applied to measured vertical velocity, not to the PWM
   command. The restored `Kd=0.15` term uses the PID's existing 1.5 Hz
   derivative filter to add vertical acceleration damping. Forward/back plus
   yaw on T5–T8 remain under left-stick control.
   During the short Select+Arm hand-off before explicit Start, the PID
   command remains enabled but all eight commands are exactly neutral.
3. Run `station_hold` (shown as `定点模式`) for the simple direct hold. Height
   uses exactly the same map-Z velocity loop and actuator-domain PID as
   `定高模式`. The left stick keeps its normal forward/yaw behavior, but its
   input changes the forward-position and heading targets; releasing it latches
   the measured two-dimensional position and heading. Independent conservative
   actuator-domain P gains close the surge, sway, and yaw-rate loops. Surge and
   sway remain limited to `0.10`; yaw has a separate `0.20` limit so every
   contributing thruster clears its measured deadband at normal stick input.
   The PID loop runs at `50 Hz`. External-IMU angular rate retains three-sample
   median rejection and uses a `0.04 s` first-order low-pass (about `4 Hz`
   cutoff); orientation filtering remains separate at `0.05 s`.
   The combined result is then scaled to the live PWM Limit. Forward motion
   retains `(-u,-u,+u,+u)` on T5--T8. Sway and heading use
   direction-specific coefficients fitted to the measured forward/reverse
   thrust curves; positive heading still keeps T5/T8 positive and T6/T7
   negative, matching the field-verified counter-clockwise direction. The sway
   loop corrects lateral drift while roll, pitch, and the full wrench allocator
   remain outside this mode. Full forward stick requests `0.30 m/s`, bounded by
   the station profile's `0.40 m/s` X limit; full yaw stick requests
   `0.60 rad/s`.
4. Run `station_hold_fast` (shown as `Station Hold Fast`) when the higher
   direct-PWM station controller is required. Full forward stick requests
   `0.40 m/s`, and the conservative `0.12` horizontal-axis cap used by normal
   station hold is removed. T5--T8 may therefore use the full live per-channel
   limit: at the default setting, each channel is independently bounded to
   `1500 +/- 200 us` rather than sharing one 200 us budget.
   This mode also commands a level roll/pitch target. The lower controller
   combines external-IMU attitude error and angular-rate damping into
   differential T1--T4 PWM while the common component continues controlling
   height. Roll/pitch differential has priority; the altitude PID is given the
   remaining PWM headroom so its integrator cannot wind up behind a saturated
   channel. The combined result is hard-limited independently on every T1--T8
   channel. Reducing the live web PWM limit still reduces this mode below
   200 us for restrained first tests.
The browser exposes independent `Mode` and `Trajectory` selectors. Both default
to `None`, which leaves the vehicle in free manual operation. Altitude hold and
the two station-hold variants are the three managed modes. Selecting
`spacial Lissajous`
automatically pairs it with station mode. Its 25-second prelude first uses a
15-second minimum-jerk translation from the measured pose to pool center
`(2.71, 1.865, 0.50) m` while holding the measured heading. It then holds the
center for 10 seconds while smoothly turning to the path's center-crossing
tangent. The
figure-eight's left-right symmetry axis is map `+F`; it then starts from the
center and publishes the full 2.30 x 1.20 x 0.10 m
Lissajous target pose, height, tangent heading, linear velocity, and angular
velocity with a 200 s period and a 10 s smooth speed ramp. It uses the direct
station controller; it does not restore the removed six-axis PID/allocation
path.

Experiments never arm the vehicle. Select PID and arm it first, then request an
allow-listed scenario:

```bash
ros2 action send_goal /runtime/run_tracking_experiment \
  robotcore_interfaces/action/RunTrackingExperiment \
  "{scenario: pose_hold, controller: pid, duration_s: 86400.0}" \
  --feedback
```

The same flow is available in the browser: choose a managed task and click
Start; the browser performs the PID Arm hand-off automatically. The action
resets the task clock, runs the configured hold/tracking
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

The dedicated `/control/rl/thruster_cmd` input accepts a future eight-output
RL policy adapter identified by `source=rl_action_adapter`.
The adapter requires exactly eight finite outputs, a measured thruster
configuration, and a non-empty `policy_layout_hash` equal to that
configuration's hash. The edge authority rejects RL by default. The retained
six-thruster WarpAUV weights are offline provenance only: they have no
deployable `policy.yaml` and must not be mapped or padded onto this vehicle.
