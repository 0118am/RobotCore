# RobotCore Project Guide

## 1. Definition

RobotCore is the system infrastructure layer for underwater robot
edge deployment.

The project uses ROS 2 as the main integration spine. Edge hardware, policy,
control, safety, logging, and UI components are decoupled through stable
ROS 2 topics, services, actions, tf, rosbag2, and ros2_control boundaries.

## 2. Architecture Principles

ROS 2 is the backbone. The operator UI is
an independent project in `../ControlInterface`; its `web_operator_node.py` remains a
ROS state bridge but has no ownership of hardware, host services, or map files.

```text
UI
  -> ROS 2 interface
     -> Runtime / Policy / Control / Safety / Logging
        -> Edge backend
```

Design rules:

- `robotcore_interfaces` owns message, service, and action contracts.
- Runtime packages depend on interfaces, not directly on Aboard.
- Edge hardware publishes real sensor and board state, then consumes the same
  control commands.
- The UI calls services/actions and subscribes to status topics. It does not
  import backend internals.

## 3. Package Boundaries

| Package | Responsibility |
| --- | --- |
| `robotcore_interfaces` | ROS 2 msg/srv/action contracts |
| `robotcore_bringup` | launch files and system configuration |
| `robotcore_runtime` | task manager, blackboard, safety events, run logging orchestration |
| `robotcore_policy` | policy registry, model runners, observation builder, action decoder |
| `robotcore_control` | thruster allocation, PWM mapping, vehicle control, safety filtering |
| `robotcore_sensors` | AprilTag/ZED/external-IMU localisation, sensor conditioning, and fixed transforms |
| `robotcore_hardware` | ros2_control hardware interface, serial/CAN/Aboard packet boundary |
| `control_interface` | browser project in `../ControlInterface`; its ROS-to-web bridge consumes installed interfaces |
| `robotcore_host_manager` | local-only systemd/device/config/log/maintenance management; not a ROS package |
| `../aquaboard` | Sole STM32 PWM 8–15, UART-v2, watchdog, failsafe, and board-status implementation |

## 4. Topic and Service Contract

Core topics:

- `/zedx/zed_node/imu/data`: ZED camera IMU for the operator HUD. ZED fuses
  it internally for VIO; it is not a separate localisation input.
- `/zedx/zed_node/rgb/color/rect/image`: rectified ZED RGB stream consumed only
  by the Isaac ROS CUDA detector.
- `/zedx/zed_node/rgb/color/rect/camera_info`: matching calibrated projection
  data used by AprilTag PnP.
- `/localization/apriltag/detections`: `tag36h11` IDs and ordered image corners
  from `isaac_ros_apriltag` with `backends=CUDA`. Its single-size pose is not
  used because `/etc/robotcore/apriltag_map.json` contains mixed Tag sizes.
- `/zedx/zed_node/rgb/color/rect/image/compressed`: native compressed operator
  video; localisation does not copy or encode display frames.
- `/localization/apriltag_pose`: the single `AprilTagPoseEstimate` input to the
  ESKF. The map localizer jointly solves all visible mapped corners using the
  calibrated CameraInfo plus each Tag's `size_m` and surveyed pose from
  `/etc/robotcore/apriltag_map.json`. Pose, covariance, quality/rejection fields,
  and `map_generation` travel together; map reload does not create a separate
  relocalization topic.
- `/localization/zed_odom`: ZED VIO local `odom -> base_link` pose and
  base-frame twist, adapted from the camera-local odometry message.
- `/hardware/aboard_imu_raw`: valid external UART8 gyro and acceleration
  samples forwarded by the A-board. No sample is published for an invalid
  frame-3 payload.
- `/sensors/external_imu`: the single calibrated external-IMU stream shared by
  the ESKF and browser. It uses `base_link` and retains the ROS specific-force
  convention (`+g` on Z at rest for a level FLU mounting).
- `/localization/fused_odom`: canonical 60 Hz position/velocity estimate and
  the only downstream odometry publisher. The ESKF fuses Tag pose, ZED VIO
  pose/velocity and calibrated UART8 IMU on one fixed-lag timeline.
- `/robot/body_state`: 60 Hz view of the same fused base pose, body-frame velocity, and
  validity. Velocity is estimated by ZED VIO and the EKF, not by finite
  differencing AprilTag poses.
- `/localization/status`: quantitative source ages, measured rates, transport
  delays, detected/mapped/inlier Tag counts, reprojection RMS, Tag/VIO
  innovation, rejection reasons, and fused covariance. The run logger persists
  this status at 1 Hz.
- `/control/thruster_cmd`: 8 normalized thruster commands.
- `/policy/body/status`: readiness and missing inputs.
- `/safety/events`: aborts, failsafe transitions, limits, and warnings.
- `/hardware/board_status`: Aboard link safety state and MCU-latched PWM
  command echoes; it is not motor/ESC feedback.

Browser-facing camera endpoints:

- `/api/camera/front.jpg`: latest front-camera snapshot.
- `/stream/camera/front.mjpg`: multipart MJPEG stream used by the operator UI.

Core services:

- `/safety/abort`: trigger an abort and force zero thruster output.
- `/policy/body/set_policy`: switch the active body policy.

Core action:

- `/runtime/run_task`: long-running task execution entry point.

## 5. Hardware Boundary

- Jetson runs ROS 2, policy inference, planning, vision, sensor bridge, and
  logging.
- Aboard receives 8 normalized thruster commands, validates packets, maps to
  PWM, handles heartbeat/failsafe state, returns board status, and forwards the
  IMU connected to its UART8 as telemetry on the shared UART6 transport.
- aCube accepts only the CRC/session/sequence UART v2 command path and maps its
  eight logical channels to physical PWM indexes 8 through 15.
- The browser publishes only `/control/candidates/manual`. It has no serial
  device parameter, UART encoder, PWM span, or physical-channel mapping; the
  central authority and the sole A-board bridge remain mandatory boundaries.
- `BoardStatus.pwm_us` is an MCU timer-latch acknowledgement. It must not be
  presented as measured ESC, motor-speed, or thrust feedback.

## 6. Policy Runtime Contract

Every policy package should contain:

- `README.md`
- `policy.yaml`
- model file or explicit dummy model marker
- input schema
- output schema

Supported runner interfaces:

| Format | Runner |
| --- | --- |
| ONNX | `onnx_runner.py` |
| PTH | `torch_runner.py` |
| MMN | `mmn_runner.py` placeholder until the real format is confirmed |
| Dummy | deterministic mock runner for integration tests |

`PolicyStatus.missing_inputs` must report readiness gaps instead of failing
silently.

## 7. Run Data Contract

Each run writes data under:

```text
data/robotcore_runs/run_YYYYMMDD_HHMMSS/
  rosbag2/
  event_log.jsonl
  policy_io/
  captures/
  configs/
```

The run folder must include enough information to replay the experiment:
rosbag2 data, safety/runtime events, policy I/O records, captured UI/sensor
artifacts, and config snapshots.

`event_log.jsonl` is a bounded operator-readable summary. High-rate repeated
streams are sampled and written through a buffered best-effort subscriber so
logging cannot backpressure control. Lossless full-rate capture, when required,
belongs in `rosbag2/`; an empty directory is not acceptance evidence.

## 8. Development Order

1. Confirm hardware mapping, sensor frames, and operating limits.
2. Validate the fail-closed ROS 2 command-authority graph.
3. Validate UART protocol, PWM mapping, watchdog, and reset behavior on aCube.
4. Validate localization and sensor timing on the Jetson.
5. Run BodyPolicy only after its required inputs pass freshness gates.
6. Run the independent operator UI and record an acceptance rosbag.
