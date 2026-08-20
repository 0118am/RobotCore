#!/usr/bin/env bash
set -euo pipefail

if [[ ${EUID} -ne 0 ]]; then
  echo "run with sudo: sudo bash scripts/install_robotcore_services.sh [options]" >&2
  exit 1
fi

repo_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
unit_root=/etc/systemd/system
config_root=/etc/robotcore
libexec_root=/usr/local/libexec
edge_env=${config_root}/edge.env

robot_workspace=${repo_root}/ros_ws
web_workspace=/home/nvidia/ControlInterface
zed_workspace=/home/nvidia/ros2_ws

usage() {
  cat <<'EOF'
Usage: sudo bash scripts/install_robotcore_services.sh [options]

Options:
  --robot-workspace PATH  Built RobotCore ROS workspace (default: this checkout/ros_ws)
  --web-workspace PATH    Built ControlInterface workspace (default: /home/nvidia/ControlInterface)
  --zed-workspace PATH    Built ZED ROS workspace (default: /home/nvidia/ros2_ws)
  -h, --help              Show this help
EOF
}

while [[ $# -gt 0 ]]; do
  case $1 in
    --robot-workspace)
      [[ $# -ge 2 ]] || { echo "--robot-workspace needs a path" >&2; exit 2; }
      robot_workspace=$2
      shift 2
      ;;
    --web-workspace)
      [[ $# -ge 2 ]] || { echo "--web-workspace needs a path" >&2; exit 2; }
      web_workspace=$2
      shift 2
      ;;
    --zed-workspace)
      [[ $# -ge 2 ]] || { echo "--zed-workspace needs a path" >&2; exit 2; }
      zed_workspace=$2
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

validate_env_path() {
  local label=$1
  local value=$2
  if [[ ! ${value} =~ ^/[A-Za-z0-9._/+:-]+$ ]]; then
    echo "${label} must be an absolute path without whitespace: ${value}" >&2
    exit 2
  fi
}

validate_env_path ROBOTCORE_WORKSPACE "${robot_workspace}"
validate_env_path CONTROL_INTERFACE_WORKSPACE "${web_workspace}"
validate_env_path ZED_WORKSPACE "${zed_workspace}"

# Refuse to install a service without the production C++ localisation graph.
# The VIO/Tag fusion executable is the deployment marker produced by the
# current workspace build.
test -r "${robot_workspace}/install/setup.bash" || {
  echo "RobotCore workspace is not built: ${robot_workspace}" >&2
  exit 1
}
test -x "${robot_workspace}/install/robotcore_sensors/lib/robotcore_sensors/vio_tag_fusion_node" || {
  echo "C++ vio_tag_fusion_node is missing from ${robot_workspace}" >&2
  exit 1
}
test -x "${robot_workspace}/install/robotcore_control_cpp/lib/robotcore_control_cpp/command_authority" || {
  echo "C++ command_authority is missing from ${robot_workspace}" >&2
  exit 1
}
test -r "${web_workspace}/install/setup.bash" || {
  echo "ControlInterface workspace is not built: ${web_workspace}" >&2
  exit 1
}
test -r "${zed_workspace}/install/setup.bash" || {
  echo "ZED workspace is not built: ${zed_workspace}" >&2
  exit 1
}
getent passwd robotcore >/dev/null || {
  echo "service account 'robotcore' does not exist" >&2
  exit 1
}

# The development installs live below a 0750 home directory. Grant only path
# traversal on that one parent; do not add the service account to the user's
# primary group or make the home directory world-readable.
if [[ ${robot_workspace} == /home/nvidia/* ||
      ${web_workspace} == /home/nvidia/* ||
      ${zed_workspace} == /home/nvidia/* ]]; then
  setfacl -m u:robotcore:--x /home/nvidia
fi

runuser -u robotcore -- test -r "${robot_workspace}/install/setup.bash" || {
  echo "robotcore cannot read ${robot_workspace}/install/setup.bash" >&2
  exit 1
}
runuser -u robotcore -- test -r "${web_workspace}/install/setup.bash" || {
  echo "robotcore cannot read ${web_workspace}/install/setup.bash" >&2
  exit 1
}
runuser -u robotcore -- test -r "${zed_workspace}/install/setup.bash" || {
  echo "robotcore cannot read ${zed_workspace}/install/setup.bash" >&2
  exit 1
}

apt-get update
apt-get install -y ros-humble-rmw-cyclonedds-cpp

install -d -m 0750 "${config_root}"
install -d -m 0755 "${unit_root}/robotcore.service.d"
install -d -m 0755 "${libexec_root}"
install -m 0755 "${repo_root}/host_manager/bin/robotcore-camera-ipc-ready" \
  "${libexec_root}/robotcore-camera-ipc-ready"
install -m 0644 "${repo_root}/host_manager/systemd/robotcore.service" "${unit_root}/robotcore.service"
install -m 0644 "${repo_root}/host_manager/systemd/control-interface.service" "${unit_root}/control-interface.service"
install -m 0644 "${repo_root}/host_manager/systemd/robotcore-host-manager.service" "${unit_root}/robotcore-host-manager.service"
install -m 0644 "${repo_root}/host_manager/systemd/robotcore-camera-ipc-ready.service" "${unit_root}/robotcore-camera-ipc-ready.service"
install -m 0644 "${repo_root}/host_manager/systemd/robotcore-performance.service" "${unit_root}/robotcore-performance.service"
install -m 0644 "${repo_root}/host_manager/systemd/robotcore-stack.target" "${unit_root}/robotcore-stack.target"
install -m 0644 "${repo_root}/host_manager/systemd/robotcore-argus.conf" \
  "${unit_root}/robotcore.service.d/argus.conf"
install -m 0644 "${repo_root}/host_manager/config/cyclonedds.xml" "${config_root}/cyclonedds.xml"
install -m 0644 "${repo_root}/host_manager/systemd/99-robotcore-dds.conf" \
  /etc/sysctl.d/99-robotcore-dds.conf

# This old override bypasses the reviewed unit and points at one developer
# workspace. edge.env is now the only workspace authority.
rm -f "${unit_root}/robotcore.service.d/tag-vio.conf"

if [[ ! -e ${edge_env} ]]; then
  install -m 0640 "${repo_root}/host_manager/systemd/edge.env.example" "${edge_env}"
fi

set_env_value() {
  local key=$1
  local value=$2
  if grep -q "^${key}=" "${edge_env}"; then
    sed -i "s|^${key}=.*|${key}=${value}|" "${edge_env}"
  else
    printf '%s=%s\n' "${key}" "${value}" >> "${edge_env}"
  fi
}

set_env_value ROBOTCORE_WORKSPACE "${robot_workspace}"
set_env_value CONTROL_INTERFACE_WORKSPACE "${web_workspace}"
set_env_value ZED_WORKSPACE "${zed_workspace}"
set_env_value ROS_LOCALHOST_ONLY 1
set_env_value ROS_DOMAIN_ID 42
set_env_value RMW_IMPLEMENTATION rmw_cyclonedds_cpp
set_env_value CYCLONEDDS_URI file:///etc/robotcore/cyclonedds.xml
set_env_value ROBOTCORE_RUN_ROOT /var/lib/robotcore/runs
set_env_value ROBOTCORE_CONFIG_ROOT /var/lib/robotcore/config
chown root:robotcore "${edge_env}" "${config_root}/cyclonedds.xml"
chmod 0640 "${edge_env}" "${config_root}/cyclonedds.xml"
install -d -o robotcore -g robotcore -m 0750 /var/lib/robotcore
install -d -o robotcore -g robotcore -m 0750 /var/lib/robotcore/runs
install -d -o root -g robotops -m 0750 /var/lib/robotcore/config
install -d -o root -g robotops -m 0750 /var/lib/robotcore/config/pid
install -d -o root -g robotops -m 0750 /var/lib/robotcore/config/pid/profiles
install -d -o root -g robotops -m 0750 /var/lib/robotcore/config/tasks
if [[ ! -e /var/lib/robotcore/config/pid/active.json ]]; then
  install -o root -g robotops -m 0640 \
    "${repo_root}/ros_ws/src/robotcore_control/config/pid/default.json" \
    /var/lib/robotcore/config/pid/active.json
fi
if [[ ! -e /var/lib/robotcore/config/pid/profiles/default.json ]]; then
  install -o root -g robotops -m 0640 \
    "${repo_root}/ros_ws/src/robotcore_control/config/pid/default.json" \
    /var/lib/robotcore/config/pid/profiles/default.json
fi
for task_file in "${repo_root}"/ros_ws/src/robotcore_runtime/config/tasks/*.json; do
  task_name=$(basename "${task_file}")
  if [[ ! -e /var/lib/robotcore/config/tasks/${task_name} ]]; then
    install -o root -g robotops -m 0640 "${task_file}" \
      "/var/lib/robotcore/config/tasks/${task_name}"
  fi
done

# Apply only RobotCore's queue tuning. Loading every host sysctl fragment here
# produces unrelated Jetson/container warnings and can obscure a real failure.
sysctl -p /etc/sysctl.d/99-robotcore-dds.conf
systemd-analyze verify \
  "${unit_root}/robotcore-performance.service" \
  "${unit_root}/robotcore-camera-ipc-ready.service" \
  "${unit_root}/robotcore-host-manager.service" \
  "${unit_root}/robotcore.service" \
  "${unit_root}/control-interface.service" \
  "${unit_root}/robotcore-stack.target"
systemctl daemon-reload

systemctl disable robotcore.service control-interface.service >/dev/null 2>&1 || true
systemctl enable robotcore-stack.target

echo "Services installed but not started. Review ${edge_env}, disarm propulsion, then run:"
echo "  sudo systemctl start robotcore-stack.target"
