#!/usr/bin/env bash
# Verify that the active ZED Link boot configuration exposes only the RTSO-3002
# i2c-7 camera path and does not load the generic cam_i2cmux overlay path.
set -euo pipefail

config_path="${EXTLINUX_CONF:-/boot/extlinux/extlinux.conf}"
boot_label="${ROBOTCORE_ZEDLINK_BOOT_LABEL:-Stereolabs}"
# Leave this empty by default so the checker validates the DTB selected by the
# boot entry.  Both the baseline RTSO-3002 DTB and its verified PCL-fixed
# variant are valid deployments.  Supplying --fdt (or ROBOTCORE_ZEDLINK_DTB) keeps
# the original strict, exact-file check for maintenance operations.
target_fdt="${ROBOTCORE_ZEDLINK_DTB:-}"
base_fdt="/boot/kernel_tegra234-p3768-0000+p3767-0000-nv-super-rtso3002-zedlink-mono-cam1-no-m2wake.dtb"
pcl_fixed_fdt="/boot/kernel_tegra234-p3768-0000+p3767-0000-nv-super-rtso3002-zedlink-mono-cam1-no-m2wake-pcl-fixed.dtb"
bad_overlay="${ROBOTCORE_ZEDLINK_BAD_OVERLAY:-/boot/tegra234-p3768-camera-zedlink-mono-sl-overlay.dtbo}"
dmesg_log=""
check_live=false

usage() {
  cat <<EOF
Usage: $0 [--config PATH] [--label LABEL] [--fdt PATH] [--bad-overlay PATH] [--dmesg-log PATH] [--live]

Checks:
  - extlinux LABEL uses the RTSO-3002 ZED Link DTB.
  - extlinux does not load the generic Stereolabs ZED Link overlay anywhere.
  - the target DTB contains only the i2c-7 ZED Link path.
  - optional dmesg log contains no i2c-9/i2c-10 ZED Link probe failures.
  - optional live device-tree contains no cam_i2cmux ZED Link nodes.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --config)
      config_path="$2"
      shift 2
      ;;
    --label)
      boot_label="$2"
      shift 2
      ;;
    --fdt)
      target_fdt="$2"
      shift 2
      ;;
    --bad-overlay)
      bad_overlay="$2"
      shift 2
      ;;
    --dmesg-log)
      dmesg_log="$2"
      shift 2
      ;;
    --live)
      check_live=true
      shift
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

require_file() {
  local path="$1"
  if [[ ! -f "${path}" ]]; then
    echo "missing file: ${path}" >&2
    exit 1
  fi
}

require_file "${config_path}"

if ! command -v dtc >/dev/null 2>&1; then
  echo "missing dtc; install device-tree-compiler to inspect DTB contents" >&2
  exit 1
fi

label_block="$(awk -v label="${boot_label}" '
  /^[[:space:]]*LABEL[[:space:]]+/ {
    if (in_target) {
      exit
    }
    in_target = ($2 == label)
  }
  in_target {
    print
  }
' "${config_path}")"

if [[ -z "${label_block}" ]]; then
  echo "fail: boot label not found: ${boot_label}" >&2
  exit 1
fi

current_fdt="$(awk '$1 == "FDT" { print $2; exit }' <<<"${label_block}")"
if [[ -n "${target_fdt}" ]]; then
  if [[ "${current_fdt}" != "${target_fdt}" ]]; then
    echo "fail: ${boot_label} does not point at ${target_fdt}" >&2
    echo "${label_block}" >&2
    exit 1
  fi
else
  case "${current_fdt}" in
    "${base_fdt}"|"${pcl_fixed_fdt}")
      target_fdt="${current_fdt}"
      ;;
    *)
      echo "fail: ${boot_label} uses an unapproved ZED Link DTB: ${current_fdt}" >&2
      echo "expected one of: ${base_fdt}, ${pcl_fixed_fdt}" >&2
      echo "${label_block}" >&2
      exit 1
      ;;
  esac
fi
require_file "${target_fdt}"
echo "ok extlinux FDT: ${target_fdt}"

if grep -Fq "${bad_overlay}" "${config_path}"; then
  echo "fail: extlinux still references bad overlay ${bad_overlay}" >&2
  grep -Fn "${bad_overlay}" "${config_path}" >&2
  exit 1
fi
echo "ok extlinux overlay: bad ZED Link overlay not referenced"

dtb_dump="$(dtc -I dtb -O dts "${target_fdt}" 2>/dev/null)"

for required in \
  "/bus@0/i2c@c250000" \
  "max9296_a@48" \
  "zedx_right_0@20" \
  "zedx_left_0@28" \
  'devname = "zedx 7-0020"' \
  'devname = "zedx 7-0028"'; do
  if ! grep -Fq "${required}" <<<"${dtb_dump}"; then
    echo "fail: DTB missing expected i2c-7 item: ${required}" >&2
    exit 1
  fi
done
echo "ok DTB contains i2c-7 ZED Link nodes"

for forbidden in \
  "cam_i2cmux" \
  "zedx_right_1@20" \
  "zedx_left_1@28" \
  "max9296_b@48"; do
  if grep -Fq "${forbidden}" <<<"${dtb_dump}"; then
    echo "fail: DTB still contains wrong ZED Link item: ${forbidden}" >&2
    exit 1
  fi
done
echo "ok DTB has no cam_i2cmux/i2c-9/i2c-10 ZED Link nodes"

if [[ -n "${dmesg_log}" ]]; then
  require_file "${dmesg_log}"
  if grep -Eq 'i2c-(9|10): .*zedx|sl_max9296 (9|10)-0048|zedx (9|10)-00(20|28)' "${dmesg_log}"; then
    echo "fail: dmesg log still contains i2c-9/i2c-10 ZED Link probes" >&2
    grep -E 'i2c-(9|10): .*zedx|sl_max9296 (9|10)-0048|zedx (9|10)-00(20|28)' "${dmesg_log}" >&2
    exit 1
  fi
  echo "ok dmesg log has no i2c-9/i2c-10 ZED Link probes"
fi

if [[ "${check_live}" == true ]]; then
  live_root="/proc/device-tree"
  if [[ ! -e "${live_root}" ]]; then
    echo "fail: live device-tree is not available at ${live_root}" >&2
    exit 1
  fi
  if [[ -e "${live_root}/bus@0/cam_i2cmux" ]]; then
    echo "fail: live device-tree still contains /bus@0/cam_i2cmux" >&2
    exit 1
  fi
  for live_node in \
    "${live_root}/bus@0/i2c@c250000/max9296_a@48" \
    "${live_root}/bus@0/i2c@c250000/zedx_right_0@20" \
    "${live_root}/bus@0/i2c@c250000/zedx_left_0@28"; do
    if [[ ! -e "${live_node}" ]]; then
      echo "fail: live device-tree missing expected node ${live_node}" >&2
      exit 1
    fi
  done
  echo "ok live device-tree has i2c-7 ZED Link nodes and no cam_i2cmux"
fi
