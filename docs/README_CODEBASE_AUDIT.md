# Codebase Audit

Audit date: 2026-06-16

## Starting State

The repository started as a minimal project containing:

- `README.md`
- `LICENSE`

No existing ROS 2 packages, MuJoCo assets, UI code, firmware code, or test
layout were present.

## Framework Added

The repository has been structured as a colcon-friendly ROS 2 Humble workspace
with explicit boundaries for:

- Interface contracts.
- Bringup and launch.
- Runtime, safety, and logging.
- Policy runtime.
- Control path.
- Sensor mocks and fusion placeholders.
- Hardware bridge skeleton.
- MuJoCo backend skeleton.
- Independent UI skeleton.
- Firmware packet/PWM/failsafe skeleton.
- Model and hardware selection documentation.

## Known Gaps

These are intentional Phase 0/1 gaps, not hidden assumptions:

- Real BlueROV2 Heavy source assets and licenses are not confirmed.
- Real underwater arm model and driver are not confirmed.
- RoboMaster Aboard PWM channel count and flashing path are not confirmed.
- MuJoCo package version is not pinned until target-machine validation.
- Jetson CUDA/cuDNN/TensorRT/PyTorch/ONNX Runtime compatibility is not tested.
- End-to-end ROS 2 execution requires Ubuntu 22.04 with ROS 2 Humble installed.

## Immediate Next Checks

- Build with `colcon build --symlink-install` on Ubuntu 22.04 + ROS 2 Humble.
- Launch `eup_edge_system.launch.py`.
- Confirm required topics with `ros2 topic list`.
- Trigger `/safety/abort`.
- Create one run folder with `scripts/robotcore_create_run_dir.py`.
