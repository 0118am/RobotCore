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
ControlInterface -> /control/manual/thruster_cmd --\
PID controller  -> /control/pid/thruster_cmd ------> command_authority
RL adapter      -> /control/rl/thruster_cmd -------/
  -> /control/thruster_cmd
  -> aboard_bridge (normalized limit and UART-v2 framing)
  -> Aquaboard synchronized PWM latch (logical 0..7 -> physical PWM 8..15)
```

The browser does not open an Aquaboard device, construct UART frames, choose a
physical PWM channel offset, or map normalized commands to microseconds. The
historically named `manual_thruster_span_us` launch argument is retained only
as the Aquaboard bridge's final `span_us` limit; it applies to every authority
source and must not be passed to the web node. `BoardStatus.pwm_us` is the
MCU-latched command echo at the timer update boundary, not ESC speed, current,
or thrust feedback.

## Aquaboard UART8 inertial telemetry

The IMU is wired to the Aquaboard's UART8. Jetson does not access that UART
directly and must not run a USB/CH340 IMU driver. The Aquaboard samples UART8 and
forwards its values in the `FF F8` telemetry stream carried over the shared
Aquaboard UART6/USB link. CRC-valid 33-byte version-2 frame 4 contains the
three-axis gyro, acceleration, native roll/pitch/yaw, acquisition tick, source
counter and separate sample/attitude-valid flags. The bridge
rejects invalid, duplicate, or backwards samples and publishes accepted samples
directly on `/sensors/external_imu`; it does not fabricate a fixed-rate stream.

Frame 5 is a separate 2 Hz runtime-budget channel. The bridge publishes it as
`/hardware/board_runtime`, including MCU CPU idle, control/UART WCET and
deadline misses, stack margins, watchdog misses, and UART error/drop counters.
It is diagnostic-only and cannot alter command or safety state.

The bridge trusts the IMU's factory-calibrated physical-unit output and native
VG/AH/MINS attitude. It applies the measured UART8 mounting conversion
(`base X=sensor Y`, `base Y=-sensor X`, `base Z=sensor Z`), corrects the native
pitch sign, and converts roll/pitch/yaw to a ROS `base_link` quaternion. It does
not estimate a second bias, remove gravity, apply another low-pass filter, or
run a software AHRS. The Status panel's
**IMU Calibration** action executes the external IMU's own saved `0x5a` gyro
calibration through Aquaboard and is rejected unless propulsion is disarmed and
all PWM outputs report neutral. `/sensors/external_imu` bypasses localization:
PID consumes its native attitude and angular rate directly, the trajectory node
uses it to latch the station-hold heading, and the browser displays it as raw
IMU telemetry. It is not subscribed by the VIO/Tag EKF.

The fixed-rate state chain is:

```text
ZED VIO pose + covariance --------> delayed EKF pose update -------------\
ZED VIO twist + covariance -------> delayed EKF twist update ------------> 60 Hz BodyState
AprilTag absolute pose -----------> static map alignment + pose update --/

external IMU attitude + gyro ----> PID/trajectory/UI directly (no EKF)
```

ZED X Mini uses one fixed 30 Hz clock for camera grab/VIO and AprilTag image
publication. Its ROS IMU publication is disabled; the external
UART8 gyro is independent of the ZED's internally fused camera IMU.

### 2026-07-30 UART8 baseline

A read-only probe of the connected Aquaboard at 115200 baud observed frame-3
telemetry at approximately 19.3 Hz with `uart8_imu_valid == false` and all
eight payload values equal to zero. No process held the serial endpoint and the
RobotCore service was inactive. Therefore the current firmware/IMU path does
not yet supply usable external IMU samples.

A separate STM32 project at `/home/nvidia/aquaboard` identified UART8 as a
Bewei IMU link. Its source at that historical baseline:

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
canonical BodyState output. Going to 200/500 Hz adds load without improving
the 60 Hz control/state contract.

This paragraph records the 2026-07-30 baseline. On 2026-08-02 the separate
STM32 source was updated to UART8 115200/100 Hz and versioned CRC frame 4,
flashed, independently read back, and verified for 1,011 consecutive samples
across multiple packed-BCD source-counter wraps. Before full system acceptance
the deployment must still:

- confirm the exact external IMU model and UART protocol;
- confirm its physical +X/+Y/+Z axes relative to ROS `base_link` FLU;
- verify the sensor persisted at 115200 baud and acknowledges 100 Hz mode
  `0x06` attitude+acceleration+gyro float output;
- verify `/sensors/external_imu` quaternion norm and physical roll/pitch/yaw
  signs; an attitude-free `0x70` fallback is not acceptable for closed-loop
  pose hold;
- restart the RobotCore service to load the rebuilt bridge's STM32-reset
  handling and repeat the end-to-end frame-4 timing check;
- reject duplicate, dropped, stale or time-regressing samples rather than
  assigning them new host timestamps.

The normal edge command needs no IMU argument:

```bash
ros2 launch robotcore_bringup robotcore_edge_system.launch.py \
  serial_port:=/dev/ttyACM0 \
  manual_thruster_span_us:=500
```

Validate all estimator inputs after launch:

```bash
ros2 topic hz /zedx/zed_node/odom
ros2 topic hz /sensors/external_imu
ros2 topic hz /robot/body_state
ros2 topic echo /localization/status --once
ros2 topic delay /zedx/zed_node/odom
ros2 topic delay /localization/apriltag/detections
```

Record the corrected stream directly with rosbag:

```bash
ros2 bag record -o calibrated_imu_bag \
  /sensors/external_imu
```

The host applies no runtime calibration stage. If the device-side gyro action is
used, keep the vehicle level and completely still until Aquaboard reports success.

## Camera/external-IMU time synchronization

The deployed RTSO-3002 DTB already configures the ZED Link MAX9296 with
`sync_mode = "master"`.  This is the preferred direction for the first hardware
integration: keep the ZED Link as trigger master and capture its frame-sync
output on the Aquaboard.  Do not switch the daemon to slave mode merely to add
a timestamp reference.

For an official ZED Link Mono card, Stereolabs documents J4 pin 6 as
`TRIG_OUT/MFP0` and J4 pin 1 as ground.  The rising edge marks the end of camera
exposure; at the configured 30 FPS the signal is 30 Hz with an 8.33 ms high
time.  Its documented 3.75 V +/- 12% level can reach 4.2 V.  Therefore:

- connect ZED sync ground and Aquaboard ground (the shared supply does not
  remove the need for an explicit signal return in the cable);
- pass `TRIG_OUT` through a 3.3 V-compatible level shifter or a verified
  divider/Schmitt input before the STM32; do not assume an arbitrary input pin
  is 5 V tolerant;
- use an unused 32-bit STM32 timer input-capture channel for the rising edge;
  TIM4 and TIM5 are already the eight-thruster PWM timers, so they must not be
  repurposed. TIM2 is a candidate only after the Aquaboard schematic and
  exposed connector pin have been verified;
- do not copy the official J4 pin numbers onto the integrated RTSO-3002 carrier
  without checking its connector routing by schematic or continuity test.

This wire supplies a hard camera-exposure event in the MCU clock domain. It
does not, by itself, trigger the external Bewei IMU. The available Bewei UART
manual documents only a cyclic 0--255 frame counter and automatic output-rate
selection; it does not document `SYNC`, `PPS`, `DRDY`, or external-trigger
operation. The current firmware also assigns `sample_tick_ms` only after the
complete UART packet has been parsed, so that field includes approximately
2.9 ms (33-byte packet) or 4.2 ms (48-byte packet) of 115200-baud serialization
plus up to one scheduler period.

The firmware follow-up, after the two physical pins are confirmed, is:

1. Run one free-running microsecond timer. Capture both the ZED rising edge and
   the UART8 frame-end/IDLE event in that timer domain.
2. Reconstruct 100 Hz IMU sample times from the sensor frame counter, using the
   UART frame-end capture to discipline phase instead of timestamping parser
   completion with `HAL_GetTick()`.
3. Forward a separate frame-sync sequence and capture timestamp to Jetson, and
   use it to anchor MCU time to the matching ZED image timestamp. Detect pulse,
   image, and IMU-counter gaps rather than silently pairing by arrival time.
4. If the exact IMU model exposes a documented hardware sync/data-ready pin,
   fan the same trigger to that input and capture its data-ready edge. That is
   the only route to hard synchronization of the physical IMU sampling instant;
   otherwise the result is hard clock anchoring plus counter-based interpolation.

The ROS wrapper must retain the SDK camera timestamp. The deployed profile sets
`general.sdk_use_monotonic_clock: true` and
`debug.use_pub_timestamps: false`, preventing NTP/system-clock steps and ROS
publication latency from becoming measurement-time errors. This complements
the wire but is not a substitute for it.

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
