# Minimum Acceptance Checklist

This file defines the first-stage system-chain acceptance target.

## Launch

- [ ] `ros2 launch eup_bringup eup_edge_system.launch.py` starts the Jetson/A-board
      edge graph and the ZED visual-inertial odometry stack.
- [ ] `ros2 launch eup_bringup eup_mujoco_system.launch.py` starts the MuJoCo
      backend path.

## Sensor and Robot State

- [ ] `/zedx/zed_node/imu/data` is published for the operator HUD. The ZED
      SDK fuses this camera IMU internally for its VIO output; no separate
      ROS IMU stream is fused by the localisation EKF.
- [ ] `/zedx/zed_node/rgb/color/rect/image` and its matching CameraInfo publish
      real-time ZED frames for AprilTag localisation.
- [ ] `/localization/apriltag/debug_image/compressed` carries the recognised-tag
      overlay selected by the browser camera panel.
- [ ] `/localization/apriltag_pose` is a mapped AprilTag measurement and
      `/localization/fused_odom` is the map-frame pose composed from AprilTag
      map-to-odom calibration and ZED VIO local odometry.
- [ ] `/localization/zed_odom` contains ZED VIO local `odom -> base_link`
      pose and base-frame twist; it is never relabelled as a map pose.
- [ ] The ZED launch publishes only its internal static camera-frame TF tree;
      it does not publish dynamic `odom` or `map` transforms.
- [ ] The raw AprilTag node does not broadcast a competing dynamic TF. The
      Tag/VIO alignment is the sole map-pose authority. Its correction gate
      rejects an inconsistent tag and continues with ZED VIO estimation.
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
