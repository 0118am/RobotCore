#!/usr/bin/env bash
# Verify the minimum Phase 1/2 topic surface after a launch file is running.
set -euo pipefail
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
front_camera_topic="${ROBOTCORE_FRONT_CAMERA_TOPIC:-/zedx/zed_node/rgb/color/rect/image}"
imu_topic="${ROBOTCORE_IMU_TOPIC:-/zedx/zed_node/imu/data}"

# Keep this list aligned with ACCEPTANCE.md.
required_topics=(
  "${imu_topic}"
  "${front_camera_topic}"
  /robot/body_state
  /robot/arm_state
  /control/thruster_cmd
  /control/arm_cmd
  /policy/body/status
  /policy/arm/status
  /safety/events
)

for topic in "${required_topics[@]}"; do
  if ros2 topic list | grep -qx "${topic}"; then
    echo "ok ${topic}"
  else
    echo "missing ${topic}" >&2
    exit 1
  fi
done

web_url="${ROBOTCORE_WEB_URL:-http://127.0.0.1:8080}"
python3 - "${web_url}/stream/camera/front.mjpg" <<'PY'
import socket
import sys
from urllib import request

url = sys.argv[1]
try:
    response = request.urlopen(url, timeout=2.0)
except Exception as exc:
    raise SystemExit(f"missing camera stream {url}: {exc}")

content_type = response.headers.get("Content-Type", "")
if "multipart/x-mixed-replace" not in content_type:
    response.close()
    raise SystemExit(f"bad camera stream content-type {content_type!r} at {url}")

payload = bytearray()
try:
    while len(payload) < 262144:
        chunk = response.read(4096)
        if not chunk:
            break
        payload.extend(chunk)
        if b"--frame" in payload and b"Content-Type: image/" in payload:
            if b"\xff\xd8" in payload or b"\x89PNG\r\n\x1a\n" in payload:
                print(f"ok {url}")
                break
    else:
        raise SystemExit(f"camera stream did not deliver a frame: {url}")
except (TimeoutError, socket.timeout):
    raise SystemExit(f"camera stream timed out waiting for a frame: {url}")
finally:
    response.close()

if b"--frame" not in payload or b"Content-Type: image/" not in payload:
    raise SystemExit(f"camera stream did not deliver a multipart image frame: {url}")
if b"\xff\xd8" not in payload and b"\x89PNG\r\n\x1a\n" not in payload:
    raise SystemExit(f"camera stream frame did not contain JPEG or PNG bytes: {url}")
PY

python3 "${script_dir}/robotcore_ui_state_check.py" --url "${web_url}/api/state"
