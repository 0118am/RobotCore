#!/usr/bin/env python3
"""Validate the deployed AprilTag payload graph from live DDS endpoints."""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from dataclasses import dataclass


os.environ.setdefault("ROS_DOMAIN_ID", "42")
os.environ.setdefault("ROS_LOCALHOST_ONLY", "1")
os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")
os.environ.setdefault("CYCLONEDDS_URI", "file:///etc/robotcore/cyclonedds.xml")

try:
    import rclpy
except ModuleNotFoundError:
    if os.environ.get("ROBOTCORE_GRAPH_CHECK_BOOTSTRAPPED") == "1":
        raise
    os.environ["ROBOTCORE_GRAPH_CHECK_BOOTSTRAPPED"] = "1"
    os.execle(
        "/bin/bash",
        "bash",
        "-c",
        "source /opt/ros/humble/setup.bash && "
        "source /home/nvidia/ros2_ws/install/setup.bash && "
        "source /home/nvidia/RobotCore/ros_ws/install/setup.bash && "
        "exec python3 /home/nvidia/RobotCore/scripts/robotcore_apriltag_graph_check.py \"$@\"",
        "robotcore-apriltag-graph-check",
        *sys.argv[1:],
        os.environ,
    )


ZED_NODE = "/zedx/zed_node"
CONVERTER_NODE = "/apriltag_cuda_rgb_converter"
DETECTOR_NODE = "/apriltag_cuda_detector"
LOCALIZER_NODE = "/apriltag_localization"
ESKF_NODE = "/fixed_lag_eskf"
UI_NODES = {"/web_operator", "/web_operator_ui"}

RAW_IMAGE = "/zedx/zed_node/rgb/color/rect/image"
CAMERA_INFO = "/zedx/zed_node/rgb/color/rect/camera_info"
CUDA_INPUT = "/localization/apriltag/cuda_input_rgb"
DETECTIONS = "/localization/apriltag/detections"
POSE = "/localization/apriltag_pose"
DETECTED_COUNT = "/localization/apriltag/detected_count"
COMPRESSED_IMAGE = "/zedx/zed_node/rgb/color/rect/image/compressed"


@dataclass(frozen=True)
class TopicContract:
    topic: str
    publishers: frozenset[str]
    subscribers: frozenset[str]


CORE_CONTRACTS = (
    TopicContract(RAW_IMAGE, frozenset({ZED_NODE}), frozenset({CONVERTER_NODE})),
    TopicContract(
        CAMERA_INFO,
        frozenset({ZED_NODE}),
        frozenset({DETECTOR_NODE, LOCALIZER_NODE}),
    ),
    TopicContract(CUDA_INPUT, frozenset({CONVERTER_NODE}), frozenset({DETECTOR_NODE})),
    TopicContract(DETECTIONS, frozenset({DETECTOR_NODE}), frozenset({LOCALIZER_NODE})),
    TopicContract(POSE, frozenset({LOCALIZER_NODE}), frozenset({ESKF_NODE})),
)


def full_node_name(name: str, namespace: str) -> str:
    namespace = namespace or "/"
    if namespace == "/":
        return f"/{name.lstrip('/')}"
    return f"/{namespace.strip('/')}/{name.lstrip('/')}"


def endpoint_nodes(endpoints) -> set[str]:
    return {
        full_node_name(endpoint.node_name, endpoint.node_namespace)
        for endpoint in endpoints
    }


def topic_endpoints(node, topic: str) -> tuple[set[str], set[str]]:
    return (
        endpoint_nodes(node.get_publishers_info_by_topic(topic)),
        endpoint_nodes(node.get_subscriptions_info_by_topic(topic)),
    )


def validate(node, require_ui: bool, require_ui_video: bool) -> tuple[list[str], dict]:
    errors: list[str] = []
    snapshot: dict[str, dict[str, list[str]]] = {}

    for contract in CORE_CONTRACTS:
        publishers, subscribers = topic_endpoints(node, contract.topic)
        snapshot[contract.topic] = {
            "publishers": sorted(publishers),
            "subscribers": sorted(subscribers),
        }
        if publishers != set(contract.publishers):
            errors.append(
                f"{contract.topic}: publishers={sorted(publishers)}, "
                f"expected={sorted(contract.publishers)}"
            )
        if subscribers != set(contract.subscribers):
            errors.append(
                f"{contract.topic}: subscribers={sorted(subscribers)}, "
                f"expected={sorted(contract.subscribers)}"
            )

    count_publishers, count_subscribers = topic_endpoints(node, DETECTED_COUNT)
    snapshot[DETECTED_COUNT] = {
        "publishers": sorted(count_publishers),
        "subscribers": sorted(count_subscribers),
    }
    if count_publishers != {LOCALIZER_NODE}:
        errors.append(
            f"{DETECTED_COUNT}: publishers={sorted(count_publishers)}, "
            f"expected={[LOCALIZER_NODE]}"
        )
    if not count_subscribers.issubset(UI_NODES):
        errors.append(
            f"{DETECTED_COUNT}: unexpected subscribers="
            f"{sorted(count_subscribers - UI_NODES)}"
        )
    if require_ui and not count_subscribers.intersection(UI_NODES):
        errors.append(f"{DETECTED_COUNT}: UI subscription is required but absent")

    video_publishers, video_subscribers = topic_endpoints(node, COMPRESSED_IMAGE)
    snapshot[COMPRESSED_IMAGE] = {
        "publishers": sorted(video_publishers),
        "subscribers": sorted(video_subscribers),
    }
    if video_publishers and video_publishers != {ZED_NODE}:
        errors.append(
            f"{COMPRESSED_IMAGE}: publishers={sorted(video_publishers)}, "
            f"expected={[ZED_NODE]} when active"
        )
    if not video_subscribers.issubset(UI_NODES):
        errors.append(
            f"{COMPRESSED_IMAGE}: unexpected subscribers="
            f"{sorted(video_subscribers - UI_NODES)}"
        )
    if require_ui_video and not (
        video_publishers == {ZED_NODE} and video_subscribers.intersection(UI_NODES)
    ):
        errors.append(
            f"{COMPRESSED_IMAGE}: active ZED-to-UI compressed branch is required"
        )

    return errors, snapshot


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--timeout", type=float, default=10.0)
    parser.add_argument("--require-ui", action="store_true")
    parser.add_argument(
        "--require-ui-video",
        action="store_true",
        help="require an active browser stream and compressed-image subscription",
    )
    parser.add_argument("--json", action="store_true", dest="as_json")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = rclpy.create_node("apriltag_graph_contract_check", enable_rosout=False)
    try:
        deadline = time.monotonic() + max(0.5, args.timeout)
        errors: list[str] = ["DDS discovery has not completed"]
        snapshot: dict = {}
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.25)
            errors, snapshot = validate(
                node,
                require_ui=args.require_ui or args.require_ui_video,
                require_ui_video=args.require_ui_video,
            )
            if not errors:
                break

        result = {
            "valid": not errors,
            "ros_domain_id": os.environ.get("ROS_DOMAIN_ID", "0"),
            "topics": snapshot,
            "errors": errors,
        }
        if args.as_json:
            print(json.dumps(result, indent=2, sort_keys=True))
        elif errors:
            print("AprilTag graph contract: FAIL", file=sys.stderr)
            for error in errors:
                print(f"- {error}", file=sys.stderr)
        else:
            print("AprilTag graph contract: PASS")
            for topic, endpoints in snapshot.items():
                print(
                    f"- {topic}: P={endpoints['publishers']} "
                    f"S={endpoints['subscribers']}"
                )
        return 0 if not errors else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
