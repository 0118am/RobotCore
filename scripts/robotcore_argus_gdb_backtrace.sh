#!/usr/bin/env bash
set -Eeuo pipefail

DIAG="${DIAG:-/usr/local/zed/tools/ZED_Diagnostic}"
OUT_DIR="${OUT_DIR:-/tmp/argus-gdb-backtrace-$(date +%Y%m%d-%H%M%S)}"
GDB="${GDB:-/usr/bin/gdb}"

if [[ "${EUID}" -ne 0 ]]; then
  echo "Run with sudo: sudo $0" >&2
  exit 1
fi

if [[ ! -x "${GDB}" ]]; then
  echo "Missing gdb: ${GDB}" >&2
  exit 1
fi

if [[ ! -x "${DIAG}" ]]; then
  echo "Missing ZED Diagnostic CLI: ${DIAG}" >&2
  exit 1
fi

mkdir -p "${OUT_DIR}"
START_TS="$(date '+%Y-%m-%d %H:%M:%S')"

cleanup() {
  systemctl unmask nvargus-daemon.service || true
  systemctl restart nvargus-daemon.service || true
}
trap cleanup EXIT

echo "[info] output: ${OUT_DIR}"
echo "[info] stopping/masking system nvargus-daemon"
systemctl stop nvargus-daemon.service
systemctl mask --runtime nvargus-daemon.service

cat >"${OUT_DIR}/nvargus.gdb" <<'GDBEOF'
set pagination off
set confirm off
set print thread-events off
handle SIGPIPE nostop noprint pass
run
printf "\n===== gdb: process stopped/exited =====\n"
info threads
thread apply all bt full
info sharedlibrary
quit
GDBEOF

echo "[info] starting nvargus-daemon under gdb"
"${GDB}" -q -x "${OUT_DIR}/nvargus.gdb" --args /usr/sbin/nvargus-daemon \
  >"${OUT_DIR}/nvargus-gdb.stdout" \
  2>"${OUT_DIR}/nvargus-gdb.stderr" &
GDB_PID="$!"

sleep 4

echo "[info] running ZED Diagnostic CLI against gdb Argus"
set +e
"${DIAG}" -c >"${OUT_DIR}/ZED_Diagnostic.stdout" 2>"${OUT_DIR}/ZED_Diagnostic.stderr"
DIAG_RC=$?
set -e
echo "${DIAG_RC}" >"${OUT_DIR}/ZED_Diagnostic.exit_code"

echo "[info] waiting for gdb to collect backtrace"
set +e
timeout 30s tail --pid="${GDB_PID}" -f /dev/null
GDB_WAIT_RC=$?
set -e
if [[ "${GDB_WAIT_RC}" -eq 124 ]]; then
  echo "[warn] gdb still running after timeout; terminating it"
  kill "${GDB_PID}" 2>/dev/null || true
fi
wait "${GDB_PID}" 2>/dev/null || true

echo "[info] collecting journal"
journalctl -b --since "${START_TS}" --no-pager \
  >"${OUT_DIR}/journal-since-start.log" || true
grep -Ei 'ZED|argus|NvPcl|OFParser|SCF|SEGV|status=11|camera|zedx|EndOfFile|No module|ISP|sl_max|tegra-cam|GMSL' \
  "${OUT_DIR}/journal-since-start.log" \
  >"${OUT_DIR}/journal-argus-zed.log" || true

echo "[info] diagnostic exit code: ${DIAG_RC}"
echo "[info] logs in ${OUT_DIR}"
echo "[hint] inspect: ${OUT_DIR}/nvargus-gdb.stdout"
