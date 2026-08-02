#!/usr/bin/env bash
# Source this file before ros2 CLI commands so they join the service graph.
if [[ ! -r /etc/robotcore/edge.env ]]; then
  echo "missing /etc/robotcore/edge.env" >&2
  return 1 2>/dev/null || exit 1
fi

set -a
# shellcheck disable=SC1091
source /etc/robotcore/edge.env
set +a
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///etc/robotcore/cyclonedds.xml
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1090
source "${ROBOTCORE_WORKSPACE}/install/setup.bash"
