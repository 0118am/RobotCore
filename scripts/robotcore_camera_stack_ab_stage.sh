#!/usr/bin/env bash
set -Eeuo pipefail

ACTION="${1:-status}"
KVER="${KVER:-$(uname -r)}"
BACKUP_DIR="${BACKUP_DIR:-/home/nvidia/camera-stack-backup-$(date +%Y%m%d-%H%M%S)}"
RESTORE_DIR="${RESTORE_DIR:-}"

ORIG_CAMERA_EXTRACT="${ORIG_CAMERA_EXTRACT:-/tmp/nvidia-l4t-camera-36.4.3-extract}"
ORIG_OOT_EXTRACT="${ORIG_OOT_EXTRACT:-/tmp/nvidia-l4t-kernel-oot-modules-36.4.3-extract}"

SYSTEM_LIB="/usr/lib/aarch64-linux-gnu/nvidia/libnvisppg.so"
ORIG_LIB="${ORIG_CAMERA_EXTRACT}/usr/lib/aarch64-linux-gnu/nvidia/libnvisppg.so"

SYSTEM_CAPTURE="/lib/modules/${KVER}/updates/drivers/platform/tegra/rtcpu/capture-ivc.ko"
SYSTEM_VI5="/lib/modules/${KVER}/updates/drivers/video/tegra/host/vi/nvhost-vi5.ko"
ORIG_CAPTURE="${ORIG_OOT_EXTRACT}/usr/lib/modules/${KVER}/updates/drivers/platform/tegra/rtcpu/capture-ivc.ko"
ORIG_VI5="${ORIG_OOT_EXTRACT}/usr/lib/modules/${KVER}/updates/drivers/video/tegra/host/vi/nvhost-vi5.ko"

usage() {
  cat <<EOF
Usage:
  sudo $0 status
  sudo $0 stage-nvidia-original
  sudo RESTORE_DIR=/path/to/backup $0 restore

This stages a reboot-required A/B test for:
  ${SYSTEM_LIB}
  ${SYSTEM_CAPTURE}
  ${SYSTEM_VI5}
EOF
}

require_root_for_write() {
  if [[ "${EUID}" -ne 0 ]]; then
    echo "Run with sudo for ${ACTION}" >&2
    exit 1
  fi
}

require_file() {
  local path="$1"
  if [[ ! -f "${path}" ]]; then
    echo "Missing required file: ${path}" >&2
    exit 1
  fi
}

print_status() {
  echo "[info] kernel: ${KVER}"
  for path in "${SYSTEM_LIB}" "${ORIG_LIB}" "${SYSTEM_CAPTURE}" "${ORIG_CAPTURE}" "${SYSTEM_VI5}" "${ORIG_VI5}"; do
    if [[ -f "${path}" ]]; then
      md5sum "${path}"
      stat -c '  %n size=%s mtime=%y' "${path}"
    else
      echo "MISSING ${path}"
    fi
  done
}

stage_original() {
  require_root_for_write
  require_file "${ORIG_LIB}"
  require_file "${ORIG_CAPTURE}"
  require_file "${ORIG_VI5}"
  require_file "${SYSTEM_LIB}"
  require_file "${SYSTEM_CAPTURE}"
  require_file "${SYSTEM_VI5}"

  mkdir -p "${BACKUP_DIR}"
  cp -a "${SYSTEM_LIB}" "${BACKUP_DIR}/libnvisppg.so.current"
  cp -a "${SYSTEM_CAPTURE}" "${BACKUP_DIR}/capture-ivc.ko.current"
  cp -a "${SYSTEM_VI5}" "${BACKUP_DIR}/nvhost-vi5.ko.current"
  md5sum "${BACKUP_DIR}"/* >"${BACKUP_DIR}/md5sum.txt"

  systemctl stop zed_x_daemon.service 2>/dev/null || true
  systemctl stop nvargus-daemon.service 2>/dev/null || true

  cp -a "${ORIG_LIB}" "${SYSTEM_LIB}"
  cp -a "${ORIG_CAPTURE}" "${SYSTEM_CAPTURE}"
  cp -a "${ORIG_VI5}" "${SYSTEM_VI5}"
  depmod -a "${KVER}"

  echo "[info] staged NVIDIA original camera stack"
  echo "[info] backup: ${BACKUP_DIR}"
  echo "[info] reboot is required before testing"
}

restore_backup() {
  require_root_for_write
  if [[ -z "${RESTORE_DIR}" ]]; then
    echo "Set RESTORE_DIR=/path/to/backup" >&2
    exit 1
  fi
  require_file "${RESTORE_DIR}/libnvisppg.so.current"
  require_file "${RESTORE_DIR}/capture-ivc.ko.current"
  require_file "${RESTORE_DIR}/nvhost-vi5.ko.current"

  systemctl stop zed_x_daemon.service 2>/dev/null || true
  systemctl stop nvargus-daemon.service 2>/dev/null || true

  cp -a "${RESTORE_DIR}/libnvisppg.so.current" "${SYSTEM_LIB}"
  cp -a "${RESTORE_DIR}/capture-ivc.ko.current" "${SYSTEM_CAPTURE}"
  cp -a "${RESTORE_DIR}/nvhost-vi5.ko.current" "${SYSTEM_VI5}"
  depmod -a "${KVER}"

  echo "[info] restored camera stack from ${RESTORE_DIR}"
  echo "[info] reboot is required before retesting"
}

case "${ACTION}" in
  status)
    print_status
    ;;
  stage-nvidia-original)
    stage_original
    ;;
  restore)
    restore_backup
    ;;
  -h|--help|help)
    usage
    ;;
  *)
    usage >&2
    exit 2
    ;;
esac
