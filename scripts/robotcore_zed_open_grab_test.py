#!/usr/bin/env python3
import argparse
import sys
import time

import pyzed.sl as sl


def main() -> int:
    parser = argparse.ArgumentParser(description="Minimal headless ZED SDK open/grab test")
    parser.add_argument("--frames", type=int, default=30)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--resolution", default="HD1200")
    args = parser.parse_args()

    print("[info] listing ZED SDK devices")
    devices = sl.Camera.get_device_list()
    for i, dev in enumerate(devices):
        print(f"[info] device[{i}]: {dev}")
    if not devices:
        print("[error] no devices returned by ZED SDK")

    init = sl.InitParameters()
    init.camera_resolution = getattr(sl.RESOLUTION, args.resolution)
    init.camera_fps = args.fps
    init.depth_mode = sl.DEPTH_MODE.NONE
    init.sdk_verbose = 1

    cam = sl.Camera()
    print(f"[info] opening camera resolution={args.resolution} fps={args.fps}")
    status = cam.open(init)
    print(f"[info] open status: {status}")
    if status != sl.ERROR_CODE.SUCCESS:
        return 2

    info = cam.get_camera_information()
    print(f"[info] camera model: {info.camera_model}")
    print(f"[info] serial: {info.serial_number}")

    runtime = sl.RuntimeParameters()
    ok = 0
    for idx in range(args.frames):
        status = cam.grab(runtime)
        print(f"[info] grab[{idx}]: {status}")
        if status == sl.ERROR_CODE.SUCCESS:
            ok += 1
        else:
            time.sleep(0.05)

    cam.close()
    print(f"[info] successful grabs: {ok}/{args.frames}")
    return 0 if ok > 0 else 3


if __name__ == "__main__":
    sys.exit(main())
