"""Static checks for the single production ZED launch path."""

from pathlib import Path

CORE_ROOT = Path(__file__).resolve().parents[1]
LAUNCH = CORE_ROOT / "ros_ws/src/robotcore_bringup/launch/robotcore_edge_system.launch.py"


def test_edge_launch_owns_the_only_zed_subprocess_command():
    launch = LAUNCH.read_text(encoding="utf-8")
    legacy = CORE_ROOT / "ros_ws/src/robotcore_runtime/robotcore_runtime/zed_camera_launcher.py"

    assert not legacy.exists()
    assert "exec ros2 launch zed_wrapper zed_camera.launch.py" in launch
    assert 'LaunchConfiguration("zed_workspace")' in launch
    assert 'LaunchConfiguration("zed_serial_number")' in launch
    assert 'LaunchConfiguration("zed_camera_id")' in launch
    assert "publish_tf:=false" in launch
    assert "enable_ipc:=false" in launch
    assert 'LaunchConfiguration("zed_params_file")' in launch
