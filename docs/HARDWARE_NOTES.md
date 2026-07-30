# Hardware Notes

Phase 0 must confirm the real edge hardware capabilities before implementation
locks in low-level details.

## Edge Boundary

Jetson Orin NX:

- Runs ROS 2 Humble.
- Runs policy inference, trajectory planning, vision processing, task planning,
  sensor bridge, and logging.
- Sends normalized 8-thruster commands to the low-level board.

RoboMaster Aboard candidate:

- Validates command packets.
- Maps normalized commands to PWM.
- Handles heartbeat, estop, failsafe, and board status.
- Returns telemetry to Jetson.

## A-board UART8 inertial telemetry

The IMU is wired to the A-board's UART8. Jetson does not access that UART
directly and must not run a USB/CH340 IMU driver. The A-board samples UART8 and
forwards its values in the `FF F8` telemetry stream carried over the shared
A-board UART6/USB link. The bridge can validate its freshness and decode the
values, but the edge localisation graph does not publish or consume this IMU.

UART8 telemetry contains gyro and acceleration, but no attitude estimate. ZED
VIO is the sole inertial localisation source: its SDK fuses the camera IMU
internally before publishing visual-inertial odometry. AprilTag supplies the
map-frame correction; the A-board IMU must not be added as a second EKF input.

The normal edge command needs no IMU argument:

```bash
ros2 launch eup_bringup eup_edge_system.launch.py \
  serial_port:=/dev/ttyACM0 \
  manual_thruster_span_us:=100 \
  manual_thruster_channel_offset:=8
```

Validate the ZED VIO inputs after launch:

```bash
ros2 topic info /zedx/zed_node/imu/data -v
ros2 topic echo /zedx/zed_node/imu/data --once
ros2 topic hz /zedx/zed_node/odom
```

## Aboard Confirmation Items

- [ ] Exact RoboMaster Aboard model.
- [ ] STM32 chip family and clock.
- [ ] PWM channel count.
- [ ] PWM frequency and signal level.
- [ ] UART/CAN/USB transport mode.
- [ ] Flashing/debugging method.
- [ ] Brownout and watchdog behavior.
- [ ] Whether 8 PWM outputs are directly available.

## ZED Link DTB Selection

RTSO-3002 maps the tested ZED Link mono camera path onto `i2c-7`, where the
camera stack should enumerate:

- `7-0048`: MAX9296 deserializer.
- `7-0020`: first ZED-X sensor endpoint.
- `7-0028`: second ZED-X sensor endpoint.

If boot logs show the kernel probing `i2c-9` and `i2c-10` under
`/bus@0/cam_i2cmux`, the generic Stereolabs overlay is being applied to the
wrong camera port mapping for this BSP. Use the RTSO-3002 DTB instead of
`/boot/tegra234-p3768-camera-zedlink-mono-sl-overlay.dtbo`:

```bash
sudo scripts/robotcore_fix_zedlink_dtb.sh --apply
sudo reboot
```

After reboot, verify that `dmesg` contains `sl_max9296 7-0048` and
`zedx 7-0020` / `zedx 7-0028`. The expected boot entry uses:

```text
FDT /boot/kernel_tegra234-p3768-0000+p3767-0000-nv-super-rtso3002-zedlink-mono-cam1-no-m2wake.dtb
```

The generic Stereolabs overlay itself contains the incorrect
`cam_i2cmux/i2c@0` and `cam_i2cmux/i2c@1` camera nodes that enumerate as
`i2c-10` and `i2c-9`. Do not load that overlay together with the RTSO-3002 DTB;
otherwise ZED Diagnostic can see the good `7-*` camera path and still fail on
the leftover `9-*` / `10-*` nodes.

Use the DT check before running ZED Diagnostic:

```bash
scripts/robotcore_zedlink_dt_check.sh --live
```

This must report that extlinux has no bad overlay reference, the RTSO-3002 DTB
has no `cam_i2cmux` nodes, and the live device-tree has `i2c-7` ZED Link nodes
with no `/bus@0/cam_i2cmux`.

For a saved dmesg capture, also assert that no stale `i2c-9` / `i2c-10` ZED
probe failures are present:

```bash
scripts/robotcore_zedlink_dt_check.sh --dmesg-log dmesg.log
```

When isolating ZED SDK stereo open failures, do not run
`zed_debug_proc` directly. Direct execution uses wrapper defaults such as
`zed2i` and does not load the ZED X config files. Use the repository probe so
the wrapper opens the detected ZED X Mini as `camera_model:=zedxm` with a
minimal feature set:

```bash
scripts/robotcore_zedx_stereo_open_probe.sh --restart-argus
```

The probe disables depth, positional tracking, mapping, object detection,
streaming, and IMU publication so the first failure stays focused on stereo
camera open and `nvargus-daemon`.

## 8-Thruster Output Options

If one Aboard cannot directly drive 8 thruster PWM outputs:

- Use an external PWM expander.
- Use two Aboard units with deterministic channel partitioning.
- Use CAN ESCs if the selected ESCs support it.
- Replace the low-level board with a controller that exposes enough PWM/CAN IO.
- Temporarily use an autopilot board during Phase 1/2 while preserving the ROS 2
  edge interface.

## Phase 0 Inference Smoke Tests

Record tested versions after running on target hardware:

| Component | Version | Result | Notes |
| --- | --- | --- | --- |
| CUDA | TBD | TBD | TBD |
| cuDNN | TBD | TBD | TBD |
| TensorRT | TBD | TBD | TBD |
| ONNX Runtime GPU | TBD | TBD | TBD |
| PyTorch wheel/container | TBD | TBD | TBD |

Fallbacks:

- NVIDIA official container.
- Vendor-provided PyTorch wheel.
- Source build only if container and wheel paths fail.
