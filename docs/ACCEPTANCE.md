# Minimum Acceptance Checklist

This file defines the first-stage system-chain acceptance target.

## Launch

- [ ] `ros2 launch robotcore_bringup robotcore_edge_system.launch.py` starts the Jetson/Aquaboard
      edge graph and the ZED visual-inertial odometry stack.

## Sensor and Robot State

- [ ] ZED SDK fuses its factory-calibrated camera IMU internally for VIO, while
      unused ZED ROS sensor publications remain disabled.
- [ ] `/zedx/zed_node/rgb/color/rect/image` and its matching CameraInfo publish
      real-time ZED frames at up to 30 Hz for AprilTag localisation.
- [ ] The host trusts the IMU's factory calibration and does not load or compute
      a second acceleration calibration. The optional operator gyro-calibration
      action executes and saves only the device's own `0x5a` procedure.
- [ ] Aquaboard UART8 runs at 115200 baud and CRC-valid version-1 frame 4 reports
      unique external-IMU samples at approximately 100 Hz. The observed
      invalid/all-zero 19.3 Hz baseline and repeated-sample forwarding are
      failures, not acceptable fallbacks.
- [ ] `/hardware/aboard_imu_raw` contains only valid frame-4 samples and
      `/sensors/external_imu` is its only corrected data output. It publishes
      calibrated angular velocity and ROS-standard, gravity-preserving specific
      force in `base_link`; `web_operator_ui` consumes this telemetry, but it is
      not integrated into position.
- [ ] The browser consumes the ZED compressed image directly; localisation does
      not copy or JPEG-encode camera frames for display.
- [ ] `/localization/apriltag/detections` is produced by
      `isaac_ros_apriltag` with `backends=CUDA`; the old OpenCV detector and
      custom VPI/PVA detector are not running.
- [ ] `/localization/apriltag_pose` (`AprilTagPoseEstimate`) is the fusion node's only
      AprilTag input. The same message carries the quality result and
      `map_generation`; reloading the map requests realignment by advancing that
      generation instead of publishing separate `pose_status` or
      `relocalize_event` topics.
- [ ] `/zedx/zed_node/odom` is consumed directly as the sole local motion source;
      no adapter or `/localization/zed_odom` duplicate is present.
- [ ] `/localization/fused_odom` and `/robot/body_state` each sustain
      59--61 Hz for at least 60 seconds, with strictly increasing source
      timestamps, no duplicate samples, and no growing DDS queue.
- [ ] `/localization/fused_odom` is the only downstream position/velocity
      odometry publisher. Before the first accepted Tag it is in `odom`; one
      quality-gated Tag establishes `map -> odom` immediately.
- [ ] `/localization/status` reports VIO/Tag/fused rates, source ages,
      transport delays, detected/mapped/inlier Tag counts, reprojection RMS,
      Tag/VIO translation and angle residuals, rejection reasons, and finite
      6x6 pose/twist covariance while sources are healthy.
- [ ] During a measured constant-speed run, body-frame linear velocity agrees
      with an independent distance/time reference within the test tolerance;
      it is not obtained by finite-differencing AprilTag detections.
- [ ] `/sensors/external_imu` remains approximately `0, 0, +9.80665 m/s²` at
      rest; no host-side zeroing/calibration or position-integration topic exists.
- [ ] The ZED launch publishes only its internal static camera-frame TF tree;
      it does not publish dynamic `odom` or `map` transforms.
- [ ] Isaac's raw single-size Tag TF is remapped to
      `/localization/apriltag/raw_tf`, never `/tf`. Tag/VIO alignment is the
      sole map-pose authority. Its correction gate rejects an inconsistent
      tag and continues with ZED VIO estimation.
- [ ] `/robot/body_state` publishes body pose, twist, and validity.

## Control

- [ ] `/control/thruster_cmd` carries 8 normalized thruster values.
- [ ] `SafetyMonitor` can abort and force zero thruster output.

## Policy Runtime

- [ ] BodyPolicy can load a policy manifest.
- [ ] `PolicyStatus.missing_inputs` reports missing readiness inputs.
- [ ] Policy input/output records are written under the active run folder.

## UI

- [ ] The independent UI displays body, policy, safety, and logging status.
- [ ] The UI front-camera panel uses `/stream/camera/front.mjpg` and receives a
      multipart image frame with JPEG or PNG payload bytes.
- [ ] The `Angular xyz` readout is sourced from `/sensors/external_imu`.
- [ ] The UI can trigger `/safety/abort`.

## Logging and Replay

- [ ] Each run creates `data/robotcore_runs/run_YYYYMMDD_HHMMSS/`.
- [ ] The run folder contains `rosbag2/`.
- [ ] The run folder contains `event_log.jsonl`.
- [ ] The run folder contains `policy_io/`.
- [ ] The run folder contains `configs/`.

## Hardware Boundary

- [ ] Jetson/Aquaboard details are isolated in `robotcore_hardware` and the Aquaboard firmware.
- [ ] A bridge or board reset cannot resume PWM until an explicit disarm and a
      later arm generation establish a new session.
