"""Contracts for managed PID profiles, task files, and experiment recording."""

import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
PID_CONFIG = ROOT / "ros_ws/src/robotcore_control/config/pid/default.json"
TASK_DIR = ROOT / "ros_ws/src/robotcore_runtime/config/tasks"
PID_NODE = ROOT / "ros_ws/src/robotcore_control/robotcore_control/six_dof_pid_node.py"
TRACKING_EXPERIMENT_NODE = (
    ROOT
    / "ros_ws/src/robotcore_runtime/robotcore_runtime/tracking_experiment_node.py"
)
TRACKING_MONITOR_NODE = (
    ROOT / "ros_ws/src/robotcore_runtime/robotcore_runtime/tracking_monitor_node.py"
)
COMMAND_AUTHORITY_NODE = (
    ROOT / "ros_ws/src/robotcore_control_cpp/src/command_authority_node.cpp"
)
SAFETY_CONFIG = ROOT / "ros_ws/src/robotcore_control/config/real_pool_safety.yaml"


def test_default_pid_document_contains_every_reloadable_gain():
    document = json.loads(PID_CONFIG.read_text(encoding="utf-8"))

    assert document["schema_version"] == 1
    assert len(document["outer_position_kp"]) == 3
    assert len(document["outer_orientation_kp"]) == 3
    assert len(document["max_linear_velocity_mps"]) == 3
    assert len(document["max_angular_velocity_rps"]) == 3
    for field in ("inner_kp", "inner_ki", "inner_kd", "integral_limit", "wrench_limit"):
        assert len(document[field]) == 6


def test_tracking_tasks_are_individual_named_pid_documents():
    task_paths = sorted(TASK_DIR.glob("*.json"))
    tasks = [
        json.loads(path.read_text(encoding="utf-8"))
        for path in task_paths
        if path.name != "record_topics.json"
    ]

    assert {task["name"] for task in tasks} == {
        "pose_hold",
        "step_x",
        "step_y",
        "step_z",
        "step_roll",
        "step_pitch",
        "step_yaw",
        "pose_lissajous_6dof",
    }
    for task in tasks:
        assert task["kind"] == "tracking_task"
        assert task["controller"] == "pid"
        assert task["duration_s"] > 0.0 or task.get("run_until_stopped") is True
        assert task["trajectory"]["trajectory_type"]
        assert (TASK_DIR / f"{task['name']}.json").is_file()


def test_pose_hold_is_manual_planar_control_with_automatic_height_hold():
    task = json.loads((TASK_DIR / "pose_hold.json").read_text(encoding="utf-8"))
    trajectory = task["trajectory"]

    assert task["label"] == "手柄操控 + 自动定高"
    assert task["run_until_stopped"] is True
    assert trajectory["trajectory_type"] == "altitude_hold"
    assert trajectory["attitude_mode"] == "hold_initial"
    assert trajectory["relative_to_initial_pose"] is False
    assert trajectory["center_z"] == 0.9
    assert trajectory["manual_vertical_speed_mps"] == 0.2


def test_managed_task_defaults_keep_initial_altitude_at_point_nine_metres():
    source = TRACKING_EXPERIMENT_NODE.read_text(encoding="utf-8")

    assert '"center_z": 0.9' in source


def test_confirmed_pool_envelope_contains_pose_hold_and_condition_limits():
    import yaml

    document = yaml.safe_load(SAFETY_CONFIG.read_text(encoding="utf-8"))
    authority = document["command_authority"]["ros__parameters"]
    trajectory = document["trajectory_command"]["ros__parameters"]

    assert authority["pool_bounds_configured"] is True
    assert authority["pool_min_xyz"] == [0.0, 0.0, 0.0]
    assert authority["pool_max_xyz"] == [5.42, 3.73, 1.0]
    assert trajectory["trajectory_limits_configured"] is True
    assert trajectory["max_linear_speed_mps"] == 0.45
    assert trajectory["max_angular_speed_rps"] == 0.55
    assert trajectory["attitude_min_rpy_deg"] == [-15.0, -15.0, -30.0]
    assert trajectory["attitude_max_rpy_deg"] == [15.0, 15.0, 30.0]

    task = json.loads((TASK_DIR / "pose_hold.json").read_text(encoding="utf-8"))
    assert task["trajectory"]["manual_vertical_speed_mps"] <= trajectory[
        "max_linear_speed_mps"
    ]

def test_rosbag_topic_list_captures_target_state_pid_output_and_safety():
    topics = set(
        json.loads((TASK_DIR / "record_topics.json").read_text(encoding="utf-8"))["topics"]
    )

    assert {
        "/robot/body_state",
        "/runtime/trajectory_target",
        "/runtime/tracking_status",
        "/control/pid/wrench",
        "/control/candidates/pid",
        "/control/thruster_cmd",
        "/control/authority/status",
        "/safety/events",
    } <= topics


def test_pid_node_exposes_the_complete_live_document_and_hash():
    source = PID_NODE.read_text(encoding="utf-8")

    assert 'self.create_subscription(BodyState, "/robot/body_state"' in source
    assert 'self.declare_parameter("control_rate_hz", 30.0)' in source
    assert 'self.create_service(GetPidConfig, "/control/pid/config", self.on_get_config)' in source
    assert "response.config_json = json.dumps(" in source
    assert "response.configuration_hash = self.configuration_hash" in source
    assert '"/control/candidates/manual"' in source
    assert 'altitude_mode = target_mode == "altitude_hold"' in source


def test_pid_candidate_stays_neutral_between_arm_and_explicit_start():
    source = PID_NODE.read_text(encoding="utf-8")

    assert 'idle_mode = target_mode == "idle"' in source
    assert 'status_message = "ready; neutral until Start"' in source


def test_tracking_action_waits_with_ros_future_instead_of_asyncio_event_loop():
    source = TRACKING_EXPERIMENT_NODE.read_text(encoding="utf-8")

    assert "asyncio.sleep" not in source
    assert "from rclpy.task import Future" in source
    assert "await self.wait_for_next_feedback(0.1)" in source
    assert "callback_group=self.wait_callback_group" in source


def test_estimated_body_state_uses_message_validity_without_stale_source_names():
    pid = PID_NODE.read_text(encoding="utf-8")
    monitor = TRACKING_MONITOR_NODE.read_text(encoding="utf-8")
    authority = COMMAND_AUTHORITY_NODE.read_text(encoding="utf-8")

    assert "state_valid or self.body.position_estimated" in pid
    assert "state_valid or self.last_body.position_estimated" in monitor
    assert "!body.state_valid && !body.position_estimated" in authority
    assert "estimated_pose_source_not_allowed" not in pid
    assert 'startswith("ZED VIO")' not in monitor
    assert "estimated localization source is not allowed" not in authority


def test_tracking_action_stops_trajectory_before_every_active_run_cleanup():
    source = TRACKING_EXPERIMENT_NODE.read_text(encoding="utf-8")

    assert 'Trigger, "/runtime/trajectory/stop"' in source
    assert source.count("await self.stop_trajectory()") == 2
    assert source.index("await self.stop_trajectory()", source.index("finally:")) < source.index(
        "await self.disarm()", source.index("finally:")
    )
