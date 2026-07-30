# RobotCore Host Manager

`host_manager` is the **local management plane** for an edge computer.  It
does not join the ROS 2 control graph and it is not a browser backend.  Its
only listener is a root-owned Unix-domain socket.  The browser operator remains
in [`../../ControlInterface/eup_ui`](../../ControlInterface/eup_ui/README.md), and continues to use ROS 2 topics,
services, and actions.

## Responsibility boundary

| Area | Owner | Entry point |
| --- | --- | --- |
| Camera, telemetry, policy status, task controls, emergency abort | `eup_ui` | browser -> HTTP/SSE -> ROS 2 |
| Thruster command safety, A-board protocol, failsafe | ROS runtime/hardware | ROS 2 -> A-board |
| Start/stop the ROS graph and web service | host manager | `robotcore-hostctl` Unix socket; ControlInterface exposes only RobotCore Start/Stop |
| Device identity and Linux permissions | host manager | udev + systemd |
| Host configuration, journald logs, run storage and rosbag freshness | host manager | read-only status / allowlisted lifecycle calls |
| AprilTag map file, atomic save and reload trigger | host manager | local socket, optional web editor |
| Software upgrade and target flashing | maintenance workflow | local, audited, physical-maintenance only |

The host manager deliberately has **no HTTP endpoint**, does not accept shell
commands, and never exposes a generic `systemctl`, `journalctl`, package
manager, or flash command.  Every socket action is an allowlisted operation in
`robotcore_host_manager/daemon.py`.

## New directory layout

```text
host_manager/
  robotcore_host_manager/ # Unix-socket daemon and safe status collectors
  bin/                    # robotcore-host-manager and robotcore-hostctl entry points
  config/                 # reviewed example configuration
  systemd/                # service and udev templates, not auto-installed
  workflows/              # human-approved upgrade and flashing runbooks
  POLICY.md               # authority and authentication boundary
  MIGRATION.md            # phased adoption from the current launch
```

This directory is intentionally not a ROS package.  It can manage the ROS
services, but it must neither publish control messages nor import `rclpy`.

## Local API

After installation, an operator in the configured socket group can run:

```bash
robotcore-hostctl status
robotcore-hostctl devices
robotcore-hostctl rosbag-status
robotcore-hostctl logs --service robot
sudo robotcore-hostctl restart --service robot
```

The daemon only permits `status`, `devices`, `rosbag-status`, `logs`, `start`,
`stop`, `restart`, `maintenance-status`, `apriltag-map`, `apriltag-upsert`, and
`apriltag-delete`. It cannot execute an upgrade or flash. AprilTag updates and
deletion are disabled until `apriltag_map.web_edit_enabled` is explicitly
enabled in root-owned configuration.

## Deployment sequence

The full migration, rollback, acceptance gates, configuration ownership, and
the treatment of update/flash operations are in [MIGRATION.md](MIGRATION.md).
Do not install the templates directly from a development checkout.  Review the
example configuration, copy it to `/etc/robotcore/host-manager.json`, then follow the
runbook one phase at a time.
