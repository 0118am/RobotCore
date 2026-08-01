#!/usr/bin/env bash
set -euo pipefail

readonly EXPECTED_DPKG_ARCH="arm64"
readonly EXPECTED_MACHINE_ARCH="aarch64"
readonly EXPECTED_OS="jammy"
readonly -a EXPECTED_FINGERPRINTS=(
  "428F5D2ACFBC9AA1F8EAC84658F4DA023E691207"
  "9EEB7195A4BD947C572666F9BBE5EA6DE681BCAB"
)
readonly KEY_URL="https://isaac.download.nvidia.com/isaac-ros/repos.key"
readonly KEYRING_PATH="/usr/share/keyrings/nvidia-isaac-ros-archive-keyring.gpg"
readonly SOURCE_PATH="/etc/apt/sources.list.d/nvidia-isaac-ros-release-3.list"
readonly PACKAGE="ros-humble-isaac-ros-apriltag"
readonly APT_LOCK_TIMEOUT_S="120"

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source_file="${script_dir}/isaac-ros-release-3.list"
temporary_dir="$(mktemp -d)"
trap 'rm -rf -- "${temporary_dir}"' EXIT

proxy_url="${ROBOTCORE_PROXY:-${HTTPS_PROXY:-${https_proxy:-}}}"
curl_proxy_args=()
apt_proxy_args=()
if [[ -n "${proxy_url}" ]]; then
  curl_proxy_args=(--proxy "${proxy_url}")
  apt_proxy_args=(
    -o "Acquire::http::Proxy=${proxy_url}"
    -o "Acquire::https::Proxy=${proxy_url}"
  )
  echo "Using proxy ${proxy_url} for curl and APT."
fi

dpkg_arch="$(dpkg --print-architecture)"
machine_arch="$(uname -m)"
if [[ "${dpkg_arch}" != "${EXPECTED_DPKG_ARCH}" || \
      "${machine_arch}" != "${EXPECTED_MACHINE_ARCH}" ]]; then
  echo "This installer targets Jetson ${EXPECTED_MACHINE_ARCH}/${EXPECTED_DPKG_ARCH}; " \
       "found ${machine_arch}/${dpkg_arch}." >&2
  exit 1
fi

os_codename="$(. /etc/os-release && printf '%s' "${VERSION_CODENAME:-}")"
if [[ "${os_codename}" != "${EXPECTED_OS}" ]]; then
  echo "This installer targets Ubuntu ${EXPECTED_OS}; found ${os_codename:-unknown}." >&2
  exit 1
fi

curl "${curl_proxy_args[@]}" -fsSL "${KEY_URL}" -o "${temporary_dir}/repos.key"
mapfile -t fingerprints < <(
  gpg --show-keys --with-colons "${temporary_dir}/repos.key" |
    awk -F: '$1 == "fpr" {print $10}'
)
for expected_fingerprint in "${EXPECTED_FINGERPRINTS[@]}"; do
  if ! printf '%s\n' "${fingerprints[@]}" | grep -Fqx "${expected_fingerprint}"; then
    echo "NVIDIA Isaac ROS key bundle is missing ${expected_fingerprint}." >&2
    printf 'Received fingerprints: %s\n' "${fingerprints[*]}" >&2
    exit 1
  fi
done

gpg --dearmor --yes \
  --output "${temporary_dir}/nvidia-isaac-ros-archive-keyring.gpg" \
  "${temporary_dir}/repos.key"

sudo install -d -m 0755 /usr/share/keyrings /etc/apt/sources.list.d
sudo install -m 0644 \
  "${temporary_dir}/nvidia-isaac-ros-archive-keyring.gpg" "${KEYRING_PATH}"
sudo install -m 0644 "${source_file}" "${SOURCE_PATH}"
sudo apt-get "${apt_proxy_args[@]}" \
  -o DPkg::Lock::Timeout="${APT_LOCK_TIMEOUT_S}" update
sudo apt-get "${apt_proxy_args[@]}" \
  -o DPkg::Lock::Timeout="${APT_LOCK_TIMEOUT_S}" install -y "${PACKAGE}"

# ROS setup scripts may inspect variables that are intentionally unset.
set +u
source /opt/ros/humble/setup.bash
set -u
ros2 pkg prefix isaac_ros_apriltag
ros2 interface show isaac_ros_apriltag_interfaces/msg/AprilTagDetectionArray

echo "Isaac ROS AprilTag CUDA runtime installed successfully."
