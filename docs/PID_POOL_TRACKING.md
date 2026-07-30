# Six-DoF PID Pool Tracking

This control path is designed for a tethered, eight-thruster vehicle. It is
fail-closed by default: the checked-in PID gains, thruster geometry, and pool
bounds are deliberately marked unconfirmed, so PID cannot arm after a fresh
checkout.

## Required measurements

Edit the installed-source equivalents of:

- `ros_ws/src/eup_control/config/real_pool_thrusters.yaml`
- `ros_ws/src/eup_control/config/real_pool_pid.yaml`
- `ros_ws/src/eup_control/config/real_pool_safety.yaml`

For every thruster record the physical channel, `base_link` position, positive
force direction, wiring/ESC sign, and a monotonic command-to-thrust curve.
Replace all placeholder values. Set `measured: true` only after a guarded
channel/sign test. The allocator rejects non-unit directions, duplicate/missing
channels, non-monotonic curves, and geometry whose wrench matrix rank is not
six.

Set `configured: true` in the PID file only after the staged single-axis tuning
has been reviewed. Put the measured map-frame pool envelope into both node
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
- fresh Tag-aligned ZED VIO body state and valid linear velocity;
- a fresh valid trajectory target inside the configured pool envelope.

Any candidate, body-state, target, or safety timeout disarms and latches a
fault. Clear the underlying abort first, then clear the authority fault and
explicitly arm again.

```bash
ros2 service call /control/authority/set \
  eup_interfaces/srv/SetControlAuthority \
  "{source: pid, arm: false, clear_fault: true}"

ros2 service call /control/authority/set \
  eup_interfaces/srv/SetControlAuthority \
  "{source: pid, arm: true, clear_fault: false}"
```

The edge launch starts the complete path:

```bash
cd /home/nvidia/RobotCore/ros_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch eup_bringup eup_edge_system.launch.py enable_web_ui:=false
```

The browser exposes the same source selection, preflight details, Arm, Disarm,
fault clear, and Abort actions. Manual sliders publish only
`/control/candidates/manual`.

## Controlled experiments

Experiments never arm the vehicle. Select PID and arm it first, then request an
allow-listed scenario:

```bash
ros2 action send_goal /runtime/run_tracking_experiment \
  eup_interfaces/action/RunTrackingExperiment \
  "{scenario: pose_lissajous_6dof, controller: pid, duration_s: 70.0}" \
  --feedback
```

The action resets the scenario clock, runs the configured hold/tracking phases,
and always disarms when it completes, is canceled, or detects an authority
fault. Scenario definitions live in
`ros_ws/src/eup_runtime/config/tracking_scenarios.yaml`.

Analyze one or more completed run directories with:

```bash
python3 scripts/analyze_tracking_runs.py \
  ros_ws/data/robotcore_runs/run_YYYYMMDD_HHMMSS \
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
configuration's hash. The edge authority rejects RL by default. The existing
six-thruster WarpAUV policy remains a MuJoCo-only artifact and must not be
enabled by changing the edge safety defaults.
