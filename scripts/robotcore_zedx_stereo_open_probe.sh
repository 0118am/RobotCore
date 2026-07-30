#!/usr/bin/env bash
# Run a minimal, correctly-parameterized ZED X stereo open probe and collect the
# Argus evidence needed to distinguish SDK/Argus failures from DT/driver issues.
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
zed_ros_ws="${ZED_ROS_WS:-/home/nvidia/ros2_ws}"
override_path="${ROBOTCORE_ZEDX_OPEN_OVERRIDE:-${repo_root}/ros_ws/src/eup_sensors/config/zedx_minimal_open.yaml}"
timeout_s="${ROBOTCORE_ZEDX_OPEN_TIMEOUT:-25s}"
serial_number="${ROBOTCORE_ZEDX_SERIAL_NUMBER:-50649148}"
camera_id="${ROBOTCORE_ZEDX_CAMERA_ID:--1}"
restart_argus=false

usage() {
  cat <<EOF
Usage: $0 [--restart-argus] [--zed-ros-ws PATH] [--override PATH] [--timeout DURATION] [--serial-number SN] [--camera-id ID]

Runs:
  1. scripts/robotcore_zedlink_dt_check.sh --live
  2. ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zedxm serial_number:=...
  3. systemctl/journalctl summary for nvargus-daemon

Use --restart-argus after a crash. It calls sudo systemctl restart
nvargus-daemon, so run it from an interactive terminal where sudo can prompt.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --restart-argus)
      restart_argus=true
      shift
      ;;
    --zed-ros-ws)
      zed_ros_ws="$2"
      shift 2
      ;;
    --override)
      override_path="$2"
      shift 2
      ;;
    --timeout)
      timeout_s="$2"
      shift 2
      ;;
    --serial-number)
      serial_number="$2"
      shift 2
      ;;
    --camera-id)
      camera_id="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if [[ ! -f "${override_path}" ]]; then
  echo "missing override YAML: ${override_path}" >&2
  exit 1
fi

if [[ ! -f "${zed_ros_ws}/install/setup.bash" ]]; then
  echo "missing ZED ROS workspace setup: ${zed_ros_ws}/install/setup.bash" >&2
  exit 1
fi

"${repo_root}/scripts/robotcore_zedlink_dt_check.sh" --live

if [[ "${restart_argus}" == true ]]; then
  sudo systemctl restart nvargus-daemon
fi

since="$(date '+%Y-%m-%d %H:%M:%S')"
probe_log="$(mktemp)"
trap 'rm -f "${probe_log}"' EXIT

echo "running ZED X open probe with serial_number=${serial_number}, camera_id=${camera_id}"

# Keep the wrapper URDF publisher enabled: it supplies the static
# zedx_camera_link -> center -> left/depth/IMU chain that positional tracking
# requires. Dynamic odom/map TFs remain disabled in the launch command.
# The probe uses SIGINT because ROS launch then shuts down its components
# cleanly; timeout escalates only after a bounded grace period.
set +e
bash -lc "
  source /opt/ros/humble/setup.bash
  source '${zed_ros_ws}/install/setup.bash'
  timeout --signal=INT --kill-after=15s '${timeout_s}' ros2 launch zed_wrapper zed_camera.launch.py \
    camera_model:=zedxm \
    camera_name:=zedx \
    serial_number:='${serial_number}' \
    camera_id:='${camera_id}' \
    publish_urdf:=true \
    publish_tf:=false \
    publish_map_tf:=false \
    publish_imu_tf:=false \
    node_log_type:=screen \
    ros_params_override_path:='${override_path}'
" 2>&1 | tee "${probe_log}"
probe_status="${PIPESTATUS[0]}"
set -e

# `timeout` deliberately terminates a healthy probe after it has had enough
# time to open the camera. Treat that exit code as success once the wrapper
# reports that the ZED node has started; genuine SDK/Argus failures below
# still take precedence.
if [[ "${probe_status}" -eq 124 ]] && grep -Fq '=== zedx started ===' "${probe_log}"; then
  probe_status=0
fi

if grep -Eq 'process has died|Camera detection timeout|CAMERA (FAILED TO SETUP|NOT DETECTED)|nvargus-daemon.service: Main process exited|Segmentation fault' "${probe_log}"; then
  probe_status=1
fi

echo
echo "zedx stereo open probe exit code: ${probe_status}"
echo
systemctl status nvargus-daemon --no-pager || true
echo
journalctl -u nvargus-daemon --since "${since}" --no-pager || true

exit "${probe_status}"
