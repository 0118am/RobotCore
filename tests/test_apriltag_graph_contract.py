"""Unit checks for the live AprilTag DDS endpoint contract validator."""

import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts/robotcore_apriltag_graph_check.py"
SPEC = importlib.util.spec_from_file_location("apriltag_graph_check", SCRIPT)
graph_check = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = graph_check
SPEC.loader.exec_module(graph_check)


def endpoint(full_name):
    namespace, name = full_name.rsplit("/", 1)
    return SimpleNamespace(node_name=name, node_namespace=namespace or "/")


def test_import_does_not_initialize_rclpy_or_replace_the_test_process():
    assert graph_check.rclpy is None


class FakeGraphNode:
    def __init__(self):
        self.publishers = {}
        self.subscribers = {}
        for contract in graph_check.CORE_CONTRACTS:
            self.publishers[contract.topic] = set(contract.publishers)
            self.subscribers[contract.topic] = set(contract.subscribers)
        self.publishers[graph_check.LOCALIZATION_STATUS] = {graph_check.FUSION_NODE}
        self.subscribers[graph_check.LOCALIZATION_STATUS] = {"/web_operator_ui"}
        self.publishers[graph_check.COMPRESSED_IMAGE] = set()
        self.subscribers[graph_check.COMPRESSED_IMAGE] = set()

    def get_publishers_info_by_topic(self, topic):
        return [endpoint(name) for name in self.publishers.get(topic, set())]

    def get_subscriptions_info_by_topic(self, topic):
        return [endpoint(name) for name in self.subscribers.get(topic, set())]


def test_exact_payload_contract_passes_without_active_video_client():
    errors, snapshot = graph_check.validate(
        FakeGraphNode(), require_ui=True, require_ui_video=False
    )

    assert errors == []
    assert snapshot[graph_check.RAW_IMAGE]["publishers"] == [graph_check.ZED_NODE]
    assert snapshot[graph_check.RAW_IMAGE]["subscribers"] == [
        graph_check.DETECTOR_NODE
    ]
    assert snapshot[graph_check.LOCALIZATION_STATUS]["publishers"] == [graph_check.FUSION_NODE]


def test_feedback_or_second_consumer_fails_the_contract():
    node = FakeGraphNode()
    node.subscribers[graph_check.RAW_IMAGE].add(graph_check.ZED_NODE)

    errors, _ = graph_check.validate(
        node, require_ui=True, require_ui_video=False
    )

    assert any(graph_check.RAW_IMAGE in error for error in errors)


def test_active_video_requires_zed_publisher_and_ui_subscriber():
    node = FakeGraphNode()
    errors, _ = graph_check.validate(
        node, require_ui=True, require_ui_video=True
    )
    assert any("active ZED-to-UI compressed branch" in error for error in errors)

    node.publishers[graph_check.COMPRESSED_IMAGE] = {graph_check.ZED_NODE}
    node.subscribers[graph_check.COMPRESSED_IMAGE] = {"/web_operator_ui"}
    errors, _ = graph_check.validate(
        node, require_ui=True, require_ui_video=True
    )
    assert errors == []
