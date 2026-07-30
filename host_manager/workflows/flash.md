# System and firmware flashing workflow

Flashing is hardware-specific and carries a recovery risk, so this repository
does not provide a remote flash RPC or a generic `flash` shell hook.

1. Obtain an approved board/image manifest containing target model, boot
   method, artifact SHA-256, operator, and rollback image.
2. Connect locally to the target recovery/programming interface.  Do not flash
   through the browser, ROS graph, or host-manager socket.
3. Stop and verify `robotcore.service` and `control-interface.service`; capture the
   pre-flash device identity and logs.  Maintain independent power through the
   full operation.
4. Use the board vendor's verified tool with the manifest's exact target.  Log
   its command output and artifact digest in the maintenance record.
5. Reboot, verify the udev device identity, start the robot service, and pass
   heartbeat, safety abort, neutral-output, sensor, rosbag, and web UI checks.
6. If verification fails, restore the documented rollback image before any
   further debugging.  Preserve the failed image/logs for analysis.
