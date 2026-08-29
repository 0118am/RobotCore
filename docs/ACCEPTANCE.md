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
- [ ] Aquaboard UART8 runs at 115200 baud and CRC-valid 33-byte version-2 frame 4 reports
      unique external-IMU samples at approximately 100 Hz. The observed
      invalid/all-zero 19.3 Hz baseline and repeated-sample forwarding are
      failures, not acceptable fallbacks.
- [ ] `aboard_bridge` publishes only CRC-valid, uniquely sequenced frame-4
      samples directly on `/sensors/external_imu`. The stream carries
      factory-calibrated angular velocity, ROS-standard gravity-preserving
      specific force, and the device's native VG/AH/MINS orientation in
      `base_link`; neither MCU nor host adds an AHRS.
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
- [ ] `/robot/body_state` sustains 59--61 Hz for at least 60 seconds, with
      strictly increasing source timestamps, no duplicate samples, and no
      growing DDS queue.
- [ ] `/robot/body_state` is the only downstream position/velocity state
      publisher. Before the first accepted Tag its frame is `odom`; one
      four-sample consistency gate establishes `map -> odom`.
- [ ] `/robot/body_state` dynamic orientation and angular velocity come only
      from `/sensors/external_imu`. ZED and Tag only update position/linear
      velocity; their orientation fields may be used for camera geometry,
      residual checking, and the fixed `map -> odom` alignment, never as a
      live attitude correction.
- [ ] `/localization/status` reports VIO/Tag/body-state rates, source ages,
      transport delays, detected/mapped/inlier Tag counts, reprojection RMS,
      Tag/VIO translation and angle residuals, rejection reasons, and finite
      6x6 pose/twist covariance while sources are healthy.
- [ ] During a measured constant-speed run, body-frame linear velocity agrees
      with an independent distance/time reference within the test tolerance;
      it is not obtained by finite-differencing AprilTag detections.
- [ ] `/sensors/external_imu` remains approximately `0, 0, +9.80665 m/s²` at
      rest; its quaternion is finite and unit length, level roll/pitch/yaw have
      the verified FLU signs, and no host-side AHRS, zeroing/calibration, or
      position-integration topic exists.
- [ ] The ZED launch publishes only its internal static camera-frame TF tree;
      it does not publish dynamic `odom` or `map` transforms.
- [ ] Isaac's raw single-size Tag TF is remapped to
      `/localization/apriltag/raw_tf`, never `/tf`. Tag/VIO alignment is the
      sole map-pose authority. Its correction gate rejects an inconsistent
      tag and continues with ZED VIO estimation.
- [ ] `/robot/body_state` publishes body pose, twist, and validity.

## Control

- [ ] `/control/thruster_cmd` carries eight direct T1..T8 actions in `[-1, 1]`;
      the bridge maps them exactly to `1250..1750 us`.
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
