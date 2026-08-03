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
- Handles heartbeat, failsafe, and board status.
- Returns telemetry to Jetson.

## Propulsion ownership boundary

There is one production propulsion path:

```text
ControlInterface manual candidate
  -> /control/candidates/manual
  -> command_authority
  -> /control/thruster_cmd
  -> aboard_bridge (normalized limit and UART-v2 framing)
  -> aCube synchronized PWM latch (logical 0..7 -> physical PWM 8..15)
```

The browser does not open an A-board device, construct UART frames, choose a
physical PWM channel offset, or map normalized commands to microseconds. The
historically named `manual_thruster_span_us` launch argument is retained only
as the A-board bridge's final `span_us` limit; it applies to every authority
source and must not be passed to the web node. `BoardStatus.pwm_us` is the
MCU-latched command echo at the timer update boundary, not ESC speed, current,
or thrust feedback.

## A-board UART8 inertial telemetry

The IMU is wired to the A-board's UART8. Jetson does not access that UART
directly and must not run a USB/CH340 IMU driver. The A-board samples UART8 and
forwards its values in the `FF F8` telemetry stream carried over the shared
A-board UART6/USB link. Frame 3 contains three-axis gyro, three-axis
acceleration, and a validity flag. The bridge publishes only valid samples as
`/hardware/aboard_imu_raw`; invalid or stale values are not fabricated into a
60 Hz stream.

The Status panel's **IMU Calibration** action is the only trigger for the
conditioning node's stationary gyro-bias calibration; the node then publishes
`/sensors/external_imu`, whose acceleration is referenced to the level,
stationary startup mean for operator display and logging. The parallel
`/sensors/external_imu_specific_force` topic preserves the expected gravity
vector for the ESKF. This startup calibration uses the configured rigid IMU
mounting and does not require VIO, a camera, or AprilTags.

The fixed-rate state chain is:

```text
AprilTag absolute map pose
             + ZED VIO pose and linear velocity
             -> /localization/aligned_vio_odom
             + calibrated UART8 angular velocity
             -> 30 Hz /localization/fused_odom
             -> 30 Hz /robot/body_state
```

ZED X Mini uses one fixed 30 Hz clock for camera grab/VIO and AprilTag image
publication, and publishes its internal IMU at 10 Hz for the HUD. The external
UART8 gyro is independent of the ZED's internally fused camera IMU.

### 2026-07-30 UART8 baseline

A read-only probe of the connected A-board at 115200 baud observed frame-3
telemetry at approximately 19.3 Hz with `uart8_imu_valid == false` and all
eight payload values equal to zero. No process held the serial endpoint and the
RobotCore service was inactive. Therefore the current firmware/IMU path does
not yet supply usable external IMU samples.

A separate, clean STM32 project at `/home/nvidia/aquaboard` identifies UART8 as a
Bewei IMU link. Its current source:

- initializes UART8 as 9600 8N1;
- selects Bewei automatic float gyro+acceleration output (`0x56 = 0x03`);
- selects 10 Hz automatic output (`0x0C = 0x02`);
- parses the six floats in command `0x70`, but ignores the four-byte sample
  counter;
- forwards the latest sample in frame 3 on an approximately 20 Hz telemetry
  cycle, which can repeat one 10 Hz sensor sample with a new host timestamp.

The [official Bewei digital protocol manual](https://www.bwsensing.com.cn/upload/userfile/IMU_VG_AH_MINS_%E6%95%B0%E5%AD%97%E8%BE%93%E5%87%BA%E5%8D%8F%E8%AE%AE%E6%89%8B%E5%86%8C.pdf)
defines 5, 10, 20, 25, 50, 100, 200, and 500 Hz output selections, although
the highest supported rate depends on the exact product. A `0x70` sample is 33
wire bytes, so 9600 baud has a theoretical ceiling below 30 Hz. The appropriate
robot target is 115200 baud and 100 Hz external-IMU samples; 60 Hz is the
canonical fused-state output. Going to 200/500 Hz adds load without improving
the 60 Hz control/state contract.

This paragraph records the 2026-07-30 baseline. On 2026-08-02 the separate
STM32 source was updated to UART8 115200/100 Hz and versioned CRC frame 4,
flashed, independently read back, and verified for 1,011 consecutive samples
across multiple packed-BCD source-counter wraps. Before full system acceptance
the deployment must still:

- confirm the exact external IMU model and UART protocol;
- confirm its physical +X/+Y/+Z axes relative to ROS `base_link` FLU;
- verify the sensor persisted at 115200 baud and acknowledges 100 Hz float
  gyro+acceleration output;
- restart the RobotCore service to load the rebuilt bridge's STM32-reset
  handling and repeat the end-to-end frame-4 timing check;
- reject duplicate, dropped, stale or time-regressing samples rather than
  assigning them new host timestamps.

The normal edge command needs no IMU argument:

```bash
ros2 launch robotcore_bringup robotcore_edge_system.launch.py \
  serial_port:=/dev/ttyACM0 \
  manual_thruster_span_us:=100
```

Validate all estimator inputs after launch:

```bash
ros2 topic info /zedx/zed_node/imu/data -v
ros2 topic echo /zedx/zed_node/imu/data --once
ros2 topic hz /zedx/zed_node/odom
ros2 topic echo /localization/external_imu_ready --once
ros2 topic hz /hardware/aboard_imu_raw
ros2 topic hz /sensors/external_imu
ros2 topic hz /sensors/external_imu_specific_force
ros2 topic hz /localization/fused_odom
ros2 topic hz /robot/body_state
ros2 topic echo /localization/status --once
ros2 topic delay /localization/zed_odom
ros2 topic delay /localization/apriltag/detections
```

Record the startup-zeroed IMU stream directly with rosbag:

```bash
ros2 bag record -o calibrated_imu_bag \
  /sensors/external_imu \
  /localization/external_imu_ready
```

Start recording before or after pressing **Calibrate**. The conditioner does
not publish `/sensors/external_imu` until calibration succeeds, so all IMU
messages in this bag use the calibrated startup baseline.

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
