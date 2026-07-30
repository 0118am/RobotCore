# Software upgrade workflow

This workflow is intentionally local and human-approved; it is not callable by
the web UI or the host-manager socket.

1. Put the vehicle in maintenance state and physically verify propulsion is
   unavailable.  Record the release ID and current service health.
2. Download/build the release in a staging directory.  Verify the approved
   artifact digest and dependency lock data before it reaches `/opt/RobotCore` or
   `/opt/ControlInterface`.
3. Export `/etc/robotcore`, the current release metadata, and the latest run/log
   directory reference.  Confirm free space on the target volume.
4. Stop `control-interface.service`, then `robotcore.service`.  Confirm the board has
   entered its normal failsafe/neutral state before touching the release.
5. Atomically switch the RobotCore and Web release pointers (or their equivalent),
   build/source RobotCore first and then the web workspace, and run offline tests.
6. Start the robot service, verify device and rosbag status through
   `robotcore-hostctl`, then start the web service.  Complete the normal edge smoke
   test including abort and camera validation.
7. Retain the prior release pointer until the acceptance period ends.  On any
   failed gate, stop services and switch the pointer back; do not attempt an
   in-place repair on the deployed release.
