# EUPSystemInfraPack Project Guide

## 1. Definition

EUPSystemInfraPack is the system infrastructure layer for underwater robot
simulation validation and edge deployment.

The first stage validates the full control chain in MuJoCo. The code structure
keeps the future migration path to Jetson Orin NX plus RoboMaster Aboard clear.

The project uses ROS 2 as the main integration spine. MuJoCo, edge hardware,
policy, control, safety, logging, and UI components are decoupled through stable
ROS 2 topics, services, actions, tf, rosbag2, and ros2_control boundaries.

## 2. Architecture Principles

ROS 2 is the backbone. MuJoCo is the first hardware backend. The operator UI is
an independent project in `../ControlInterface`; its `web_operator_node.py` remains a
ROS state bridge but has no ownership of hardware, host services, or map files.

```text
UI
  -> ROS 2 interface
     -> Runtime / Policy / Control / Safety / Logging
        -> MuJoCo backend
        -> Edge backend
```

Design rules:

- `eup_interfaces` owns message, service, and action contracts.
- Runtime packages depend on interfaces, not directly on MuJoCo or Aboard.
- MuJoCo publishes simulated sensor and robot state, then consumes control
  commands.
- Edge hardware publishes real sensor and board state, then consumes the same
  control commands.
- The UI calls services/actions and subscribes to status topics. It does not
  import backend internals.

## 3. Package Boundaries

| Package | Responsibility |
| --- | --- |
| `eup_interfaces` | ROS 2 msg/srv/action contracts |
| `eup_bringup` | launch files and system configuration |
| `eup_runtime` | task manager, blackboard, safety events, run logging orchestration |
| `eup_policy` | policy registry, model runners, observation builder, action decoder |
| `eup_control` | thruster allocation, PWM mapping, arm command routing, safety filtering |
| `eup_sensors` | AprilTag/ZED/external-IMU localisation, sensor conditioning, and fixed transforms |
| `eup_hardware` | ros2_control hardware interface, serial/CAN/Aboard packet boundary |
| `eup_mujoco_env` | MuJoCo model assets, sensor publisher, actuator subscriber |
| `eup_ui` | browser project in `../ControlInterface`; its ROS-to-web bridge consumes installed interfaces |
| `robotcore_host_manager` | local-only systemd/device/config/log/maintenance management; not a ROS package |
| `eup_firmware` | STM32 packet, PWM, heartbeat, failsafe, board status skeleton |

## 4. Topic and Service Contract

Core topics:

- `/clock`: simulation time from MuJoCo backend.
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
- `/localization/apriltag_pose`: absolute mapped AprilTag pose measurement.
  The map localizer jointly solves all visible mapped corners using each
  Tag's `size_m` and surveyed pose from `/etc/robotcore/apriltag_map.json`.
- `/localization/zed_odom`: ZED VIO local `odom -> base_link` pose and
  base-frame twist, adapted from the camera-local odometry message.
- `/hardware/aboard_imu_raw`: valid external UART8 gyro and acceleration
  samples forwarded by the A-board. No sample is published for an invalid
  frame-3 payload.
- `/sensors/external_imu`: stationary-bias-corrected external gyro. Orientation
  and acceleration are marked unavailable to the estimator until mounting and
  gravity handling are validated.
- `/localization/aligned_vio_odom`: event-driven map-frame pose from AprilTag
  map-to-odom alignment plus ZED VIO, retaining the ZED base-frame twist.
- `/localization/fused_odom`: canonical 30 Hz map-frame estimate. It uses the
  aligned AprilTag/ZED pose and linear velocity plus calibrated UART8 angular
  velocity.
- `/robot/body_state`: 30 Hz fused base pose, body-frame velocity, and
  validity. Velocity is estimated by ZED VIO and the EKF, not by finite
  differencing AprilTag poses.
- `/localization/status`: quantitative source ages, measured rates, transport
  delays, detected/mapped/inlier Tag counts, reprojection RMS, Tag/VIO
  innovation, rejection reasons, and fused covariance. The run logger persists
  this status at 1 Hz.
- `/robot/arm_state`: arm joint state and validity.
- `/robot/thruster_state`: 8-thruster normalized and PWM feedback.
- `/control/thruster_cmd`: 8 normalized thruster commands.
- `/control/arm_cmd`: arm joint or end-effector command.
- `/policy/body/status`, `/policy/arm/status`: readiness and missing inputs.
- `/safety/events`: aborts, failsafe transitions, limits, and warnings.
- `/hardware/board_status`: Aboard or mock board heartbeat.

Browser-facing camera endpoints:

- `/api/camera/front.jpg`: latest front-camera snapshot.
- `/stream/camera/front.mjpg`: multipart MJPEG stream used by the operator UI.

Core services:

- `/safety/abort`: trigger an abort and force zero thruster output.
- `/policy/body/set_policy`: switch the active body policy.
- `/policy/arm/set_policy`: switch the active arm policy.

Core action:

- `/runtime/run_task`: long-running task execution entry point.

## 5. Backend Boundary

MuJoCo backend:

- Loads MJCF assets.
- Publishes `/clock`, sensor state, body state, arm state, and thruster state.
- Subscribes to `/control/thruster_cmd` and `/control/arm_cmd`.
- May later be replaced by or integrated with `mujoco_ros2_control`.

Edge backend:

- Jetson runs ROS 2, policy inference, planning, vision, sensor bridge, and
  logging.
- Aboard receives 8 normalized thruster commands, validates packets, maps to
  PWM, handles heartbeat/failsafe/estop, returns board status, and forwards the
  IMU connected to its UART8 as telemetry on the shared UART6 transport.
- Aboard details remain Phase 0 confirmation items.

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

## 8. Development Order

1. Phase 0: confirm versions, models, and hardware unknowns.
2. Phase 1: build the ROS 2 mock graph.
3. Phase 2: attach MuJoCo backend.
4. Phase 3: run BodyPolicy and ArmPolicy through the common runtime.
5. Phase 4: run the independent operator UI.
6. Phase 5: implement Aboard packet, PWM, mock serial, and firmware skeleton.
7. Phase 6: validate end-to-end MuJoCo to UI demo.
