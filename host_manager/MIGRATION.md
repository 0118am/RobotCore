# Web and local-management separation migration

This plan preserves the current browser UI while separating host authority from
the robot graph.  Do each phase on a bench with propulsion disconnected or the
vehicle mechanically safe.

## Phase 0 — baseline and rollback point

1. Record the current image revision, `systemctl` state, device symlinks,
   `ros2 node list`, and a successful browser `/api/state` response.
2. Run the existing edge launch unchanged and confirm abort, board heartbeat,
   camera stream, and `scripts/robotcore_topic_check.sh`.
3. Keep the existing launch command as the rollback path.  The new
   `enable_web_ui` launch argument defaults to `true`, so this phase changes no
   existing developer workflow.

**Exit gate:** the baseline evidence is stored with the release record.

## Phase 1 — establish identities and durable paths

1. Create a non-login `robotcore` service account and a `robotops` operator group.
2. Mount or create `/var/lib/robotcore/runs` with adequate capacity and configure
   `ROBOTCORE_RUN_ROOT` to it.  Keep rosbag files off the root filesystem.
3. Install the reviewed udev rule from `systemd/99-robotcore-aboard.rules` only after
   confirming the Aquaboard VID/PID/serial.  Change the edge configuration to
   use the stable `/dev/robotcore/aboard` path.
4. Copy and tailor `config/host-manager.example.json` to
   `/etc/robotcore/host-manager.json`, then set owner `root:root`, mode `0640`.
   The managed AprilTag map is `/etc/robotcore/apriltag_map.json` by default. Keep
   its directory group `robotops` and map mode `0640` with group `robotops` so
   approved manual ROS operators can read it. Keep web editing disabled until
   the coordinate convention and access path have
   been verified. Preserve or create the selected map file before starting the
   robot service: its path must exactly match both `apriltag_map.path` and
   `ROBOTCORE_APRILTAG_MAP_FILE`. The service checks that this file is readable
   before it starts the localisation node, so it cannot silently fall back to
   the package sample map.

**Exit gate:** reboot twice; device identity, permissions, and run-storage
mount remain stable.  No direct `/dev/ttyUSB*` reference remains in deployment
configuration.

## Phase 2 — split process supervision

1. Install `robotcore.service`, `control-interface.service`,
   `robotcore-performance.service`, `robotcore-stack.target`, and
   `robotcore-host-manager.service` from `systemd/`. They are templates: replace
   `/opt/RobotCore` and `/opt/ControlInterface`, then source the reviewed
   `/etc/robotcore/edge.env` values first.
2. Start `robotcore-stack.target`; it applies the Jetson performance profile,
   then starts `robotcore.service` with `enable_web_ui:=false` and starts
   `control-interface.service` as a separate process. The default web bind is
   loopback, and direct manual serial PWM stays disabled.
4. Enable the host manager last.  Verify its socket cannot be opened by a user
   outside `robotops`.
5. When AprilTag web editing is required, enable
   `apriltag_map.web_edit_enabled`, add the `robotcore` web-service account to the
   configured socket group, and test one benign map upsert. The robot service
   receives the managed file through `ROBOTCORE_APRILTAG_MAP_FILE`.

**Exit gate:** stopping/restarting the web service does not restart the robot
graph; restarting the robot service does not grant the web service hardware or
host-management rights.  The browser still shows camera and ROS state.

For AprilTag setup, use the web panel or `robotcore-hostctl apriltag-upsert` /
`robotcore-hostctl apriltag-delete` only while the vehicle is disarmed. Confirm
the saved `position_m` is the tag
centre, `size_m` is the black-square edge, and `rpy_deg` follows the convention
in [APRILTAG_MAP.md](APRILTAG_MAP.md). Verify the debug image and localization
pose before enabling normal operation.

## Phase 3 — observability and rosbag monitoring

1. Point `run_root` and rosbag recording at `/var/lib/robotcore/runs`.
2. Configure the rosbag run root and freshness threshold in the host-manager
   JSON.  The daemon watches active `.db3`/`.mcap` files as well as final
   `metadata.yaml` files.
3. Test `robotcore-hostctl rosbag-status` during recording, after an intentional
   recorder stop, and with insufficient free space.  Alerting should consume
   this status, not parse browser state.
4. Use `robotcore-hostctl logs --service robot` for bounded journald excerpts; use
   normal journal retention and logrotate policy for long-term storage.

**Exit gate:** a stale or absent rosbag is visible locally and does not affect
the safety control loop.

## Phase 4 — controlled updates and flashing

1. Follow [workflows/upgrade.md](workflows/upgrade.md) for software artifacts.
2. Follow [workflows/flash.md](workflows/flash.md) for Jetson/Aquaboard images.
3. Keep both workflows outside the daemon RPC surface.  They require a local
   maintainer to verify artifact digest, stop/disarm the robot, capture a
   backup, and record results before restoring services.

**Exit gate:** a failed update or flash can boot the previous image or restore
the previous application release without modifying the web UI contract.

## Rollback

Disable the stack target and its services, restore the previous RobotCore/Web release directory, and run
the prior `ros2 launch robotcore_bringup robotcore_edge_system.launch.py` command.  Do not
remove logs, run directories, or the device rule during an incident; preserve
them for diagnosis.
