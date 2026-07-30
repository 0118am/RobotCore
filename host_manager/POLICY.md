# Host management policy

## Trust zones

1. **Browser zone (`eup_ui`)**: untrusted network clients.  It may observe ROS
   state and call the existing, safety-reviewed ROS interfaces.  It has no
   Unix-socket access and no credentials for Linux service management.
2. **Robot service zone (`robotcore.service`)**: the `robotcore` account owns the
   ROS graph and is the only process granted A-board device access.  The web
   service runs as the same non-root account but uses ROS controls only.
3. **Host-manager zone (`robotcore-host-manager.service`)**: root-owned, local Unix
   socket, command allowlist, audit log via journald.  Socket access is granted
   only to the `robotops` group.
4. **Maintenance zone**: upgrade and flashing require a local console, an
   approved artifact manifest, a stopped/disarmed robot service, and a
   human-recorded maintenance ticket.  They are not daemon RPCs.

## Non-negotiable controls

- Bind the web UI to `127.0.0.1` by default.  If remote access is needed, use
  an authenticated TLS reverse proxy or VPN; do not expose port 8080 directly.
- Set `manual_thruster_serial_port` to an empty value in the web service.  Only
  the A-board bridge may write the serial device in normal operation.
- `/etc/robotcore/host-manager.json` must be `root:root`, mode `0640`; only root may
  change service names, device paths, and maintenance flags.
- `/etc/robotcore/apriltag_map.json` is `root:robotops`, mode `0640`; approved
  local operators may read the map for direct ROS development, but only the
  root-owned host manager may write it.
- The socket is `root:robotops`, mode `0660`; membership in `robotops` is an
  explicit operational privilege, not a web-login role.
- The daemon returns log excerpts and health metadata only.  It does not return
  configuration secrets or arbitrary files.
- AprilTag edits are an exception with a narrow schema: ID, black-square size,
  map position, and RPY only. A deletion accepts only the numeric ID. Both
  operations write only the configured managed-map path atomically and are
  disabled unless `web_edit_enabled` is set by root. The browser never selects
  a filesystem path.
- Enabling the browser editor authorizes any authenticated operator who can use
  the existing web UI to change localization geometry. Enable it only on the
  loopback/TLS-or-VPN deployment path, and disable it after tank setup.
- `robotcore-hostctl stop/restart` is an operational action.  The socket grants it
  only to the explicitly administered `robotops` group; deployments that need a
  stricter split should expose status through a second read-only socket rather
  than broadening browser privileges.

## Ownership

| Asset | Writable by |
| --- | --- |
| `/etc/robotcore/*.json`, systemd units, udev rules | root / release process |
| `/var/lib/robotcore/runs` | `robotcore` runtime; host manager reads health only |
| `/var/log` and journald | systemd/journald |
| A-board serial device | `robotcore` service and approved local operators via `dialout` |
| Web static assets | release process; read-only at runtime |
