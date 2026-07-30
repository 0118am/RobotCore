#!/usr/bin/env bash
set -Eeuo pipefail

ACTION="${1:-status}"
WORK_DIR="${WORK_DIR:-/home/nvidia/zedlink-dtb-tests}"
EXTLINUX="${EXTLINUX:-/boot/extlinux/extlinux.conf}"
LABEL="${LABEL:-Stereolabs-pcl-position-test}"
BACKUP="${BACKUP:-/boot/extlinux/extlinux.conf.pcl-test-backup}"
FINAL_DTB="${FINAL_DTB:-/boot/kernel_tegra234-p3768-0000+p3767-0000-nv-super-rtso3002-zedlink-mono-cam1-no-m2wake-pcl-fixed.dtb}"

usage() {
  cat <<EOF
Usage:
  $0 status
  sudo $0 stage-position-rear
  sudo $0 promote-position-rear
  sudo $0 restore

stage-position-rear creates a test DTB where:
  tegra-camera-platform/modules/module1/position = "rear"

promote-position-rear makes that DTB the FDT for the normal Stereolabs entry
and sets DEFAULT back to Stereolabs.

It keeps module badges unchanged so the existing zedx_ar0234.isp lookup can still work.
EOF
}

require_root() {
  if [[ "${EUID}" -ne 0 ]]; then
    echo "Run with sudo for ${ACTION}" >&2
    exit 1
  fi
}

current_fdt() {
  awk '
    $1 == "LABEL" && $2 == "Stereolabs" { in_label=1; next }
    in_label && $1 == "LABEL" { exit }
    in_label && $1 == "FDT" { print $2; exit }
  ' "${EXTLINUX}"
}

line_from_label() {
  local key="$1"
  awk -v key="${key}" '
    $1 == "LABEL" && $2 == "Stereolabs" { in_label=1; next }
    in_label && $1 == "LABEL" { exit }
    in_label && $1 == key {
      sub(/^[ \t]*/, "")
      print
      exit
    }
  ' "${EXTLINUX}"
}

print_status() {
  echo "[info] extlinux: ${EXTLINUX}"
  echo "[info] current DEFAULT:"
  awk '$1 == "DEFAULT" { print "  " $0 }' "${EXTLINUX}" || true
  echo "[info] Stereolabs FDT: $(current_fdt)"
  if [[ -f "${FINAL_DTB}" ]]; then
    echo "[info] final DTB:"
    md5sum "${FINAL_DTB}"
  else
    echo "[info] final DTB not installed: ${FINAL_DTB}"
  fi
  echo "[info] test work dir: ${WORK_DIR}"
  if [[ -f "${WORK_DIR}/pcl-position-rear.dtb" ]]; then
    md5sum "${WORK_DIR}/pcl-position-rear.dtb"
  fi
  if grep -q "^LABEL ${LABEL}$" "${EXTLINUX}" 2>/dev/null; then
    echo "[info] test label exists: ${LABEL}"
  else
    echo "[info] test label not installed"
  fi
}

stage_position_rear() {
  require_root

  local fdt
  fdt="$(current_fdt)"
  if [[ -z "${fdt}" || ! -f "${fdt}" ]]; then
    echo "Could not find Stereolabs FDT from ${EXTLINUX}: ${fdt}" >&2
    exit 1
  fi

  mkdir -p "${WORK_DIR}"
  local src_dts="${WORK_DIR}/current.dts"
  local test_dts="${WORK_DIR}/pcl-position-rear.dts"
  local test_dtb="${WORK_DIR}/pcl-position-rear.dtb"

  dtc -I dtb -O dts -o "${src_dts}" "${fdt}"
  perl -0pe 's/(module1\s*\{.*?position\s*=\s*)"front";/${1}"rear";/s' \
    "${src_dts}" >"${test_dts}"

  if ! grep -A12 -n 'module1 {' "${test_dts}" | grep -q 'position = "rear";'; then
    echo "Failed to patch module1 position to rear" >&2
    exit 1
  fi

  dtc -I dts -O dtb -o "${test_dtb}" "${test_dts}"
  install -m 0644 "${test_dtb}" "/boot/pcl-position-rear-zedlink-test.dtb"

  if [[ ! -f "${BACKUP}" ]]; then
    cp -a "${EXTLINUX}" "${BACKUP}"
  fi

  local linux_line initrd_line append_line
  linux_line="$(line_from_label LINUX)"
  initrd_line="$(line_from_label INITRD)"
  append_line="$(line_from_label APPEND)"

  if [[ -z "${linux_line}" || -z "${initrd_line}" || -z "${append_line}" ]]; then
    echo "Could not copy LINUX/INITRD/APPEND from Stereolabs extlinux entry" >&2
    exit 1
  fi

  if ! grep -q "^LABEL ${LABEL}$" "${EXTLINUX}"; then
    {
      printf '\nLABEL %s\n' "${LABEL}"
      printf '\tMENU LABEL Stereolabs PCL position test\n'
      printf '\t%s\n' "${linux_line}"
      printf '\tFDT /boot/pcl-position-rear-zedlink-test.dtb\n'
      printf '\t%s\n' "${initrd_line}"
      printf '\t%s\n' "${append_line}"
    } >>"${EXTLINUX}"
  fi

  sed -i "s/^DEFAULT .*/DEFAULT ${LABEL}/" "${EXTLINUX}"
  sync

  echo "[info] staged DTB PCL position test"
  echo "[info] original FDT: ${fdt}"
  echo "[info] test FDT: /boot/pcl-position-rear-zedlink-test.dtb"
  echo "[info] extlinux backup: ${BACKUP}"
  echo "[info] reboot is required before testing"
}

ensure_position_rear_dtb() {
  local src_fdt="$1"
  mkdir -p "${WORK_DIR}"

  local src_dts="${WORK_DIR}/current.dts"
  local test_dts="${WORK_DIR}/pcl-position-rear.dts"
  local test_dtb="${WORK_DIR}/pcl-position-rear.dtb"

  dtc -I dtb -O dts -o "${src_dts}" "${src_fdt}"
  perl -0pe 's/(module1\s*\{.*?position\s*=\s*)"front";/${1}"rear";/s' \
    "${src_dts}" >"${test_dts}"

  if ! grep -A12 -n 'module0 {' "${test_dts}" | grep -q 'position = "front";'; then
    echo "Unexpected module0 position; refusing to promote" >&2
    exit 1
  fi
  if ! grep -A12 -n 'module1 {' "${test_dts}" | grep -q 'position = "rear";'; then
    echo "Failed to patch module1 position to rear" >&2
    exit 1
  fi

  dtc -I dts -O dtb -o "${test_dtb}" "${test_dts}"
}

promote_position_rear() {
  require_root

  local source_fdt
  source_fdt="$(current_fdt)"
  if [[ -z "${source_fdt}" || ! -f "${source_fdt}" ]]; then
    echo "Could not find Stereolabs FDT from ${EXTLINUX}: ${source_fdt}" >&2
    exit 1
  fi

  if [[ ! -f "${BACKUP}" ]]; then
    cp -a "${EXTLINUX}" "${BACKUP}"
  fi

  ensure_position_rear_dtb "${source_fdt}"
  install -m 0644 "${WORK_DIR}/pcl-position-rear.dtb" "${FINAL_DTB}"

  python3 - "$EXTLINUX" "$FINAL_DTB" <<'PY'
import pathlib
import sys

extlinux = pathlib.Path(sys.argv[1])
final_dtb = sys.argv[2]
lines = extlinux.read_text().splitlines()
out = []
in_stereolabs = False
for line in lines:
    stripped = line.split()
    if len(stripped) >= 2 and stripped[0] == "DEFAULT":
        out.append("DEFAULT Stereolabs")
        continue
    if len(stripped) >= 2 and stripped[0] == "LABEL":
        in_stereolabs = stripped[1] == "Stereolabs"
        out.append(line)
        continue
    if in_stereolabs and stripped and stripped[0] == "FDT":
        prefix = line[: len(line) - len(line.lstrip())]
        out.append(f"{prefix}FDT {final_dtb}")
        continue
    out.append(line)
extlinux.write_text("\n".join(out) + "\n")
PY
  sync

  echo "[info] promoted fixed DTB to normal Stereolabs entry"
  echo "[info] source FDT: ${source_fdt}"
  echo "[info] final FDT: ${FINAL_DTB}"
  echo "[info] extlinux backup: ${BACKUP}"
  echo "[info] reboot is required"
}

restore_extlinux() {
  require_root
  if [[ ! -f "${BACKUP}" ]]; then
    echo "Missing backup: ${BACKUP}" >&2
    exit 1
  fi
  cp -a "${BACKUP}" "${EXTLINUX}"
  sync
  echo "[info] restored ${EXTLINUX} from ${BACKUP}"
  echo "[info] reboot is required"
}

case "${ACTION}" in
  status)
    print_status
    ;;
  stage-position-rear)
    stage_position_rear
    ;;
  promote-position-rear)
    promote_position_rear
    ;;
  restore)
    restore_extlinux
    ;;
  -h|--help|help)
    usage
    ;;
  *)
    usage >&2
    exit 2
    ;;
esac
