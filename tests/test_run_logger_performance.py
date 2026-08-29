"""Checks that acceptance summaries cannot become a control-loop I/O sink."""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
LOGGER = ROOT / "ros_ws/src/robotcore_runtime/robotcore_runtime/run_logger.py"


def test_run_logger_buffers_one_open_file_and_bounds_repeated_streams():
    source = LOGGER.read_text(encoding="utf-8")

    assert "buffering=64 * 1024" in source
    assert "self.event_log_handle.write" in source
    assert 'separators=(",", ":")' in source
    assert '(self.run_dir / "event_log.jsonl").open(' in source
    assert "if self.event_log_handle is None:" in source
    assert 'self.declare_parameter("thruster_log_rate_hz", 20.0)' in source
    assert 'self.declare_parameter("authority_log_rate_hz", 10.0)' in source
    assert 'self.declare_parameter("pid_log_rate_hz", 10.0)' in source
    assert "time.monotonic_ns()" in source
    assert "reliability=ReliabilityPolicy.BEST_EFFORT" in source
    assert "depth=1" in source


def test_safety_and_experiment_events_force_a_buffer_flush():
    source = LOGGER.read_text(encoding="utf-8")

    assert "flush=bool(msg.abort_active)" in source
    assert source.count('"tracking_experiment",') == 2
    assert source.count("flush=True,") >= 3
    assert "self.stop_rosbag()" in source
    assert "self.event_log_handle.close()" in source


def test_task_snapshot_uses_only_the_managed_runtime_document():
    source = LOGGER.read_text(encoding="utf-8")

    assert "def task_path(self, task_name):" in source
    assert "task_path = self.task_path(task_name)" in source
    assert 'Path(str(self.get_parameter("task_config_dir").value)) / filename' in source
    assert 'get_package_share_directory("robotcore_runtime")' not in source
