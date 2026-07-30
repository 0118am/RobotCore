#!/usr/bin/env bash
set -Eeuo pipefail

ORIG_LIB_DIR="${ORIG_LIB_DIR:-/tmp/nvidia-camera-original-lib}"
ORIG_LIB="${ORIG_LIB_DIR}/libnvisppg.so"
DIAG="${DIAG:-/usr/local/zed/tools/ZED_Diagnostic}"
OUT_DIR="${OUT_DIR:-/tmp/argus-libnvisppg-ab-test-$(date +%Y%m%d-%H%M%S)}"

if [[ "${EUID}" -ne 0 ]]; then
  echo "Run with sudo: sudo $0" >&2
  exit 1
fi

if [[ ! -f "${ORIG_LIB}" ]]; then
  echo "Missing ${ORIG_LIB}" >&2
  echo "Create it first from the extracted nvidia-l4t-camera deb." >&2
  exit 1
fi

mkdir -p "${OUT_DIR}"

cleanup() {
  if [[ -n "${ARGUS_PID:-}" ]] && kill -0 "${ARGUS_PID}" 2>/dev/null; then
    kill "${ARGUS_PID}" 2>/dev/null || true
    wait "${ARGUS_PID}" 2>/dev/null || true
  fi
  systemctl unmask nvargus-daemon.service || true
  systemctl restart nvargus-daemon.service || true
}
trap cleanup EXIT

echo "[info] output: ${OUT_DIR}"
echo "[info] original lib:"
md5sum "${ORIG_LIB}" | tee "${OUT_DIR}/original-lib.md5"
echo "[info] system lib:"
md5sum /usr/lib/aarch64-linux-gnu/nvidia/libnvisppg.so | tee "${OUT_DIR}/system-lib.md5"

START_TS="$(date '+%Y-%m-%d %H:%M:%S')"

echo "[info] stopping system nvargus-daemon"
systemctl stop nvargus-daemon.service
systemctl mask --runtime nvargus-daemon.service

echo "[info] starting manual nvargus-daemon with LD_LIBRARY_PATH=${ORIG_LIB_DIR}"
LD_LIBRARY_PATH="${ORIG_LIB_DIR}:${LD_LIBRARY_PATH:-}" /usr/sbin/nvargus-daemon \
  >"${OUT_DIR}/manual-nvargus.stdout" \
  2>"${OUT_DIR}/manual-nvargus.stderr" &
ARGUS_PID="$!"

sleep 2
if ! kill -0 "${ARGUS_PID}" 2>/dev/null; then
  echo "[error] manual nvargus-daemon exited early" >&2
  cat "${OUT_DIR}/manual-nvargus.stderr" >&2 || true
  exit 1
fi

echo "[info] manual nvargus-daemon pid: ${ARGUS_PID}"
grep -F 'libnvisppg.so' "/proc/${ARGUS_PID}/maps" \
  | tee "${OUT_DIR}/manual-nvargus-libnvisppg.maps" || true

echo "[info] running ZED Diagnostic CLI against manual Argus"
set +e
"${DIAG}" -c >"${OUT_DIR}/ZED_Diagnostic.stdout" 2>"${OUT_DIR}/ZED_Diagnostic.stderr"
DIAG_RC=$?
set -e
echo "${DIAG_RC}" >"${OUT_DIR}/ZED_Diagnostic.exit_code"

if [[ -n "${ARGUS_PID:-}" ]] && ! kill -0 "${ARGUS_PID}" 2>/dev/null; then
  echo "[info] manual nvargus-daemon exited during diagnostic" \
    | tee "${OUT_DIR}/manual-nvargus-exit-state.txt"
fi

echo "[info] collecting journal"
journalctl -b --since "${START_TS}" --no-pager \
  >"${OUT_DIR}/journal-since-start.log" || true

echo "[info] matching Argus/ZED lines:"
grep -Ei 'ZED|argus|NvPcl|OFParser|SCF|SEGV|status=11|camera|zedx|EndOfFile|No module|ISP|sl_max|tegra-cam|GMSL' \
  "${OUT_DIR}/journal-since-start.log" \
  | tee "${OUT_DIR}/journal-argus-zed.log" || true

echo "[info] done; diagnostic exit code: ${DIAG_RC}"
echo "[info] logs in ${OUT_DIR}"
