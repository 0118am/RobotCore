#!/usr/bin/env bash
# Print the exact version probes that Phase 0 should run on the target machine.
# The script is intentionally non-invasive: it records commands to execute
# rather than installing GPU/ROS dependencies.
set -euo pipefail

echo "RobotCore Phase 0 smoke test placeholder"
echo "Record the following on the target Ubuntu 22.04 / ROS 2 Humble machine:"
echo "- lsb_release -a"
echo "- ros2 --version"
echo "- python3 -c 'import mujoco; print(mujoco.__version__)'"
echo "- python3 -c 'import torch; print(torch.__version__)'"
echo "- python3 -c 'import onnxruntime as ort; print(ort.get_available_providers())'"
echo "- trtexec --version"
