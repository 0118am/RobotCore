# RobotCore

`RobotCore` owns every non-browser concern for the underwater robot: ROS 2
runtime, sensors, localization, control, hardware transport, models,
host operations, diagnostics, and AprilTag configuration.
The browser project lives separately in [`../ControlInterface`](../ControlInterface/README.md).
The sole Aquaboard firmware source is the sibling `../aquaboard` repository; this
repository intentionally contains no second actuator protocol implementation.

Naming and ownership are authoritative: `Aboard` is the hardware brand,
`Aquaboard` is the embedded controller product, and stable ROS identifiers use
the `aboard` brand prefix. AprilTag map parsing, joint PnP, quality gating, pose
publication, and VIO/AprilTag map-alignment fusion are built and launched from this repository.
Only the upstream GPU image conversion and tag detection stages come from
Isaac ROS.

## Layout

```text
RobotCore/
  ros_ws/
    src/                 # interfaces, bringup, runtime, policy, control, sensors, hardware
  host_manager/          # systemd, udev, configuration, logs, upgrades, rosbag, AprilTag map
  models/                # policy artifacts
  scripts/               # device and deployment diagnostics
  docs/                  # system/acceptance/hardware documentation
  tests/                 # driver and hardware tests
```

`robotcore_interfaces` is the installed contract between this workspace and the
browser-side `web_operator_node.py`. The UI node stays in `ControlInterface` for an
efficient ROS-to-web state bridge, but it owns neither devices nor host files.

## Build and run

Build RobotCore first:

```bash
cd /path/to/RobotCore/ros_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Then build the web package against RobotCore's installed interfaces:

```bash
cd /path/to/ControlInterface
source /path/to/RobotCore/ros_ws/install/setup.bash
colcon build --base-paths control_interface --symlink-install
source install/setup.bash
```

The authoritative edge workflow is the manual launch from `RobotCore/ros_ws`:

The production AprilTag detector is `isaac_ros_apriltag` release 3.2 with its
CUDA backend. On JetPack 6.2 / ROS 2 Humble, install the matching NVIDIA binary
package in the runtime environment before building RobotCore. The checked
installer verifies NVIDIA's signing-key fingerprint before adding the source:

```bash
cd /path/to/RobotCore
bash scripts/install_isaac_ros_apriltag.sh
```

```bash
cd /path/to/RobotCore/ros_ws
source /opt/ros/humble/setup.bash
source /home/nvidia/ros2_ws/install/setup.bash
source install/setup.bash
ros2 launch robotcore_bringup robotcore_edge_system.launch.py \
  serial_port:=/dev/robotcore/aboard \
  apriltag_tag_map_file:=/etc/robotcore/apriltag_map.json
```

Build and source only `RobotCore/ros_ws/{build,install,log}` for this workflow.
Running `colcon build` from the `RobotCore` repository root creates a second,
stale-prone install space that the manual launch does not use. The ZED launcher
starts the fixed camera directly; no second camera command or Aquaboard IMU gate
is required.

The normal validation-vehicle edge launch starts the PID/tracking processes
against the confirmed thruster model and pool envelope. Authority,
localisation freshness, target validity and explicit operator Arm remain
independent fail-closed gates. Pass `enable_pool_tracking:=false` for a
sensor/manual-only deployment.

Detailed operating contracts are in [docs/PROJECT_GUIDE.md](docs/PROJECT_GUIDE.md).
The measured architecture/performance review and C++ rewrite thresholds are in
[docs/PERFORMANCE_ARCHITECTURE.md](docs/PERFORMANCE_ARCHITECTURE.md).
The fail-closed real-pool PID workflow is documented in
[docs/PID_POOL_TRACKING.md](docs/PID_POOL_TRACKING.md).
