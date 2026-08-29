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
RL enter it through `/control/manual/thruster_cmd`, `/control/pid/thruster_cmd`,
and `/policy/body/action`, respectively. A source can change only while
disarmed. PID arming also requires:

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
   A `0.10 s` low-pass filter is applied to measured vertical velocity, not to
   the PWM command. Following the observed height overshoot, the outer height
   gain is `0.25 1/s` and the velocity PID is `Kp=1.4`, `Ki=0.45`, `Kd=0.22`;
   the PID's existing 1.5 Hz derivative filter provides vertical acceleration
   damping. Forward/back plus yaw on T5–T8 remain under left-stick control.
   During the short Select+Arm hand-off before explicit Start, the PID
   command remains enabled but all eight commands are exactly neutral.
3. Run `station_hold` (shown as `定点模式`) for the simple direct hold. Height
   uses exactly the same map-Z velocity loop and actuator-domain PID as
   `定高模式`. The left stick keeps its normal forward/yaw behavior, but its
   input changes the forward-position and heading targets; releasing it latches
   the measured two-dimensional position and heading. Independent conservative
   actuator-domain velocity control closes surge and sway; yaw rate uses bounded
   PI. Surge and yaw integration is reset on mode/trajectory-phase changes and
   rolled back when the combined mixer saturates. Sway integral is disabled for
   the current A/B profile. Normal station mode retains its `0.12` horizontal
   effort limit and yaw has a separate `0.20` limit.
   The PID loop runs at `50 Hz`. External-IMU angular rate rejects isolated
   outliers, fits the latest five timestamped samples, predicts only the bounded
   `0.04 s` command-application horizon, and uses a `0.01 s` low-pass. The
   effective `0.13 s` pitch response lag is not treated as pure prediction
   time. Orientation filtering remains separate at `0.03 s`; its roll/pitch
   come from the external IMU while absolute yaw comes from `BodyState`, so a
   native magnetic-heading jump cannot enter the heading loop or target latch.
   After forward-run testing, the current sway A/B profile uses `Kp=1.0` and
   `Ki=0.0`; removing retained lateral effort avoids the observed side-to-side
   hunting at target crossings. Heading uses `0.90 1/s` angle-to-rate gain and
   `Kp=0.70`, `Ki=0.12` in the rate loop. A stationary 5-degree heading error
   therefore requests just over the measured 25 us deadband before integral
   action, rather than entering the previous slow final approach.
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
   `0.40 m/s`. Right-stick left/right commands body-frame lateral velocity;
   releasing it latches measured horizontal position. Right-stick up/down
   continues to command vertical velocity, and left-stick horizontal retains
   yaw control. The fast-mode sway correction is independently capped at
   `0.30 m/s`, instead of inheriting the active PID profile's conservative
   lateral-speed cap. Its dedicated sway velocity loop uses `Kp=1.8`, while
   normal station hold retains `Kp=1.0`; both keep sway integral disabled.
   During straight lateral input, automatic yaw correction
   is capped at `0.10` to prevent the measured sway/yaw coupled oscillation;
   explicit left-stick yaw retains the normal `0.20` authority. The
   conservative `0.12` horizontal-effort cap used by
   normal station hold is also removed. T5--T8 may therefore use the full live
   per-channel limit: at the default setting, each channel is independently
   bounded to `1500 +/- 250 us` rather than sharing one aggregate budget.
   This mode also commands a level roll/pitch target. An explicit outer loop
   converts attitude error to a bounded roll/pitch-rate setpoint; the inner loop
   converts rate error to differential T1--T4 PWM. Roll uses a softened
   `1.30 1/s` outer gain to damp forward-motion rocking. Pitch uses
   `0.80 1/s`, a `0.12 rad/s` rate cap, and an inner PI loop with
   `Kp=0.35`, `Ki=0.20`.
   Integral effort is capped at `0.10` (50 us), conditionally frozen by the
   shared attitude limit, and reset on mode/phase or surge-direction changes.
   Direction-specific
   surge-to-pitch decoupling (`0.08` forward, `0.06` reverse, capped at `0.06`)
   uses the actually applied surge effort and is combined before the shared
   attitude limit. Roll/pitch differential has priority; the altitude PID is
   given the remaining PWM headroom so its integrator cannot wind up behind a
   saturated channel. One station mixer now produces T1--T8 without overwriting
   a previously generated vertical command. The combined result is hard-limited
   independently on every channel. Reducing the live web PWM limit still
   reduces this mode below 250 us for restrained first tests.
5. Select `spatial_lissajous`, `circle`, `racetrack`,
   or `straight_line` only as the target trajectory. A trajectory owns no
   controller gains or actuator path. `Station Hold` and `Station Hold Fast`
   select PID; `RL Policy` selects the deployed RL controller. Every automatic
   path publishes the same controller-independent position, velocity,
   attitude, and feed-forward target contract.
   Its 25-second prelude first uses a 15-second minimum-jerk translation from
   the measured pose to that path's deterministic start point while holding
   measured heading. The start-approach phase publishes an explicit phase
   marker: both surge and sway use the planned velocity plus position
   correction without the normal software velocity caps, and either station
   mode receives the full live horizontal PWM authority. The unified
   operator-selected PWM bound and roll/pitch/yaw safety limits are never
   bypassed. The vehicle then remains at that fixed start point for 10 seconds
   while smoothly aligning with the initial path heading before tracking
   begins. Circle and racetrack commands follow their path tangent; the
   out-and-back straight line holds map-forward heading to avoid a 180-degree
   heading step at each turnaround. The packaged tasks use a 10 s smooth speed
   ramp; their periods remain task-configurable (the three newly packaged paths
   default to 100 s).
The browser exposes independent `Mode` and `Trajectory` selectors. Both default
to `None`, which leaves the vehicle in free manual operation. Selecting a
trajectory never changes `Mode`, and selecting a mode never changes
`Trajectory`. Every automatic path is startable with `Station Hold`,
`Station Hold Fast`, or `RL Policy`; the selected mode alone determines the
controller.

Experiments never arm the vehicle. Select PID and arm it first, then request an
allow-listed scenario:

```bash
ros2 action send_goal /runtime/run_tracking_experiment \
  robotcore_interfaces/action/RunTrackingExperiment \
  "{scenario: pose_hold, controller: pid, control_mode: altitude_hold, duration_s: 86400.0}" \
  --feedback
```

The same flow is available in the browser: choose a managed task and click
Start; the browser performs the PID Arm hand-off automatically. The action
resets the task clock, runs the configured hold/tracking
phases, and always disarms when it completes, is canceled, or detects an
authority fault. Task definitions are individual JSON files under
`/home/nvidia/ControlInterface/control_interface/config/tasks/`; each goal
reloads its file, so editing a task does not require restarting the action
server.

Starting an accepted experiment creates exactly one directory:

```text
/home/nvidia/robotcore_logs/runs/run_<timestamp>_<task>/
  configs/                 exact PID, task, thruster and safety snapshots
  rosbag2/tracking/        full-rate rosbag2 SQLite recording
  event_log.jsonl          bounded, operator-readable summaries and markers
  rosbag2.log              recorder output
```

The topic allow-list is
`/home/nvidia/ControlInterface/control_interface/config/tasks/record_topics.json`.
The bag is the primary evidence because it preserves source messages and ROS
timestamps. JSONL is a convenient index and quick-analysis input, not a
replacement for the bag.

Analyze one or more completed run directories with:

```bash
python3 scripts/analyze_tracking_runs.py \
  /home/nvidia/robotcore_logs/runs/run_YYYYMMDD_HHMMSS_task \
  --output tracking_report
```

To export every rosbag/event record on one nanosecond timeline and create a
causally aligned PWM/target table without replaying control topics:

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/RobotCore/ros_ws/install/setup.bash
python3 scripts/export_merged_control_log.py \
  /home/nvidia/robotcore_logs/runs/run_YYYYMMDD_HHMMSS_task
```

The exporter writes `all_records_merged.csv` and `pwm_target_merged.csv` below
`/home/nvidia/robotcore_logs/exports/<run_name>/`. The original run stays
read-only. PWM rows use the latest target/tracking/command whose `header.stamp`
is not later than the MCU status stamp, and include the alignment age in
milliseconds.

The report contains CSV/JSON metrics, trajectory delay, step settling time,
six-axis pose/velocity/error figures, thruster figures, and a multi-run summary
PNG. A run passes only when position RMSE is at most 0.15 m, orientation
geodesic RMSE is at most 10 degrees, saturation is at most 5%, valid samples
are at least 99%, and no abort or authority fault occurred.

## RL boundary

`t60_policy` publishes its eight raw model outputs on `/policy/body/action`, and
`command_authority` copies those values directly into canonical T1-T8 actions.
The deployed model was trained in the physical vehicle's channel and polarity,
so there is no RL thruster adapter, permutation, or sign map. The bridge
performs the physical conversion `PWM_us = 1500 + 250 * action`. The shared
authority PWM limit remains the final safety bound for RL, PID, and manual
sources: it multiplies all eight actions by the fixed live ratio
`pwm_limit_us / 250`, preserving allocation ratios without independently
clipping channels.
