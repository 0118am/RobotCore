#!/usr/bin/env bash
# Switch the Jetson boot entry from the generic Stereolabs ZED Link overlay to
# the RTSO-3002 DTB that maps the connected ZED Link camera onto i2c-7. The
# bad overlay is removed from every boot entry so it cannot recreate i2c-9/10.
set -euo pipefail

config_path="${EXTLINUX_CONF:-/boot/extlinux/extlinux.conf}"
boot_label="Stereolabs"
target_fdt="/boot/kernel_tegra234-p3768-0000+p3767-0000-nv-super-rtso3002-zedlink-mono-cam1-no-m2wake.dtb"
bad_overlay="/boot/tegra234-p3768-camera-zedlink-mono-sl-overlay.dtbo"
apply=false

usage() {
  cat <<EOF
Usage: $0 [--apply] [--config PATH] [--label LABEL] [--fdt PATH] [--remove-overlay PATH]

By default this prints the extlinux.conf diff without changing the system.
Use --apply on the target Jetson to write the change after reviewing the diff.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --apply)
      apply=true
      shift
      ;;
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
    --remove-overlay)
      bad_overlay="$2"
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

if [[ ! -f "${config_path}" ]]; then
  echo "missing extlinux config: ${config_path}" >&2
  exit 1
fi

if [[ "${apply}" == true && ! -e "${target_fdt}" ]]; then
  echo "target DTB does not exist: ${target_fdt}" >&2
  exit 1
fi

tmp_path="$(mktemp)"
trap 'rm -f "${tmp_path}"' EXIT

awk \
  -v label="${boot_label}" \
  -v fdt="${target_fdt}" \
  -v overlay="${bad_overlay}" '
function is_label_line(line) {
  return line ~ /^[[:space:]]*LABEL[[:space:]]+/
}

function flush_missing_fdt() {
  if (in_target && !seen_fdt) {
    print "\tFDT " fdt
  }
}

{
  if (is_label_line($0)) {
    flush_missing_fdt()
    in_target = ($2 == label)
    seen_label = seen_label || in_target
    seen_fdt = 0
  }

  if (in_target && $0 ~ /^[[:space:]]*FDT[[:space:]]+/) {
    print "\tFDT " fdt
    seen_fdt = 1
    next
  }

  if ($0 ~ /^[[:space:]]*OVERLAYS[[:space:]]+/) {
    changed = 0
    output = ""
    for (i = 2; i <= NF; i++) {
      if ($i == overlay) {
        changed = 1
        continue
      }
      output = output (output == "" ? "" : " ") $i
    }
    if (changed) {
      if (output != "") {
        print "\tOVERLAYS " output
      }
      next
    }
  }

  print
}

END {
  flush_missing_fdt()
  if (!seen_label) {
    exit 3
  }
}
' "${config_path}" > "${tmp_path}" || {
  status=$?
  if [[ "${status}" -eq 3 ]]; then
    echo "boot label not found in ${config_path}: ${boot_label}" >&2
  fi
  exit "${status}"
}

if cmp -s "${config_path}" "${tmp_path}"; then
  echo "no change needed: ${config_path}"
  exit 0
fi

diff -u "${config_path}" "${tmp_path}" || true

if [[ "${apply}" != true ]]; then
  echo
  echo "dry-run only; rerun with --apply to update ${config_path}"
  exit 0
fi

backup_path="${config_path}.bak.$(date -u +%Y%m%dT%H%M%SZ)"
cp "${config_path}" "${backup_path}"
install -m 0644 "${tmp_path}" "${config_path}"

echo "updated ${config_path}"
echo "backup saved to ${backup_path}"
echo "reboot, then verify that dmesg shows sl_max9296 7-0048 and zedx 7-0020/7-0028"
echo "also verify that no i2c-9/i2c-10 ZED Link nodes remain"
