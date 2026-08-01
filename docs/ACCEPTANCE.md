# Minimum Acceptance Checklist

This file defines the first-stage system-chain acceptance target.

## Launch

- [ ] `ros2 launch eup_bringup eup_edge_system.launch.py` starts the Jetson/A-board
      edge graph and the ZED visual-inertial odometry stack.
- [ ] `ros2 launch eup_bringup eup_mujoco_system.launch.py` starts the MuJoCo
      backend path.

## Sensor and Robot State

- [ ] `/zedx/zed_node/imu/data` publishes at approximately 10 Hz for the
      operator HUD. The ZED SDK fuses the camera IMU internally independently
      of this ROS publication rate.
- [ ] `/zedx/zed_node/rgb/color/rect/image` and its matching CameraInfo publish
      real-time ZED frames at up to 30 Hz for AprilTag localisation.
- [ ] A-board UART8 runs at 115200 baud and frame 3 reports unique external-IMU
      samples with its valid flag set at approximately 100 Hz. The observed
      invalid/all-zero 19.3 Hz baseline and repeated-sample forwarding are
      failures, not acceptable fallbacks.
- [ ] `/hardware/aboard_imu_raw` contains only valid frame-3 samples and
      `/localization/external_imu_ready` becomes true after a stationary
      calibration. `/sensors/external_imu` then publishes calibrated angular
      velocity with strictly increasing timestamps and source sample IDs.
- [ ] The browser consumes the ZED compressed image directly; localisation does
      not copy or JPEG-encode camera frames for display.
- [ ] `/localization/apriltag/detections` is produced by
      `isaac_ros_apriltag` with `backends=CUDA`; the old OpenCV detector and
      custom VPI/PVA detector are not running.
- [ ] `/localization/apriltag_pose` is a mapped AprilTag measurement and
      `/localization/aligned_vio_odom` is the event-driven map-frame pose
      composed from AprilTag map-to-odom calibration and ZED VIO local
      odometry.
- [ ] `/localization/zed_odom` contains ZED VIO local `odom -> base_link`
      pose and base-frame twist; it is never relabelled as a map pose.
- [ ] `/localization/fused_odom` and `/robot/body_state` each sustain
      57--63 Hz for at least 60 seconds, with strictly increasing source
      timestamps, no duplicate samples, and no growing DDS queue.
- [ ] During a measured constant-speed run, body-frame linear velocity agrees
      with an independent distance/time reference within the test tolerance;
      it is not obtained by finite-differencing AprilTag detections.
- [ ] External IMU acceleration remains disabled in the EKF until a physical
      FLU axis test, mounting-rotation measurement, gravity-removal test, and
      stationary/noise calibration all pass.
- [ ] The ZED launch publishes only its internal static camera-frame TF tree;
      it does not publish dynamic `odom` or `map` transforms.
- [ ] Isaac's raw single-size Tag TF is remapped to
      `/localization/apriltag/raw_tf`, never `/tf`. Tag/VIO alignment is the
      sole map-pose authority. Its correction gate rejects an inconsistent
      tag and continues with ZED VIO estimation.
- [ ] `/robot/body_state` publishes body pose, twist, and validity.
- [ ] `/robot/arm_state` publishes arm joint state after the real arm driver is connected.

## Control

- [ ] `/control/thruster_cmd` carries 8 normalized thruster values.
- [ ] `/control/arm_cmd` controls the simplified arm command path.
- [ ] `SafetyMonitor` can abort and force zero thruster output.

## Policy Runtime

- [ ] BodyPolicy can load a policy manifest.
- [ ] ArmPolicy can load a policy manifest.
- [ ] `PolicyStatus.missing_inputs` reports missing readiness inputs.
- [ ] Policy input/output records are written under the active run folder.

## UI

- [ ] The independent UI displays body, arm, policy, safety, and logging status.
- [ ] The UI front-camera panel uses `/stream/camera/front.mjpg` and receives a
      multipart image frame with JPEG or PNG payload bytes.
- [ ] The `Angular xyz` readout is sourced from `/zedx/zed_node/imu/data`.
- [ ] The UI can trigger `/safety/abort`.

## Logging and Replay

- [ ] Each run creates `data/robotcore_runs/run_YYYYMMDD_HHMMSS/`.
- [ ] The run folder contains `rosbag2/`.
- [ ] The run folder contains `event_log.jsonl`.
- [ ] The run folder contains `policy_io/`.
- [ ] The run folder contains `configs/`.

## Backend Migration

- [ ] MuJoCo and edge hardware share the same ROS 2 interface contracts.
- [ ] Jetson/Aboard details are isolated in `eup_hardware` and `eup_firmware`.
