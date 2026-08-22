"""Operator-armed, allow-listed tracking experiment action server."""

from __future__ import annotations

import json
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.task import Future
from rcl_interfaces.srv import SetParameters
from std_srvs.srv import Trigger

from robotcore_interfaces.action import RunTrackingExperiment
from robotcore_interfaces.msg import ControlAuthorityStatus, PidStatus, TrajectoryTarget
from robotcore_interfaces.srv import SetControlAuthority, StartRun, StopRun


class TrackingExperimentNode(Node):
    """Configure a safe scenario, report progress, and always disarm at exit."""

    ALLOWED_PARAMETER_TYPES = {
        "trajectory_type": Parameter.Type.STRING,
        "attitude_mode": Parameter.Type.STRING,
        "relative_to_initial_pose": Parameter.Type.BOOL,
        "center_x": Parameter.Type.DOUBLE,
        "center_y": Parameter.Type.DOUBLE,
        "center_z": Parameter.Type.DOUBLE,
        "hold_before_motion_s": Parameter.Type.DOUBLE,
        "amp_x": Parameter.Type.DOUBLE,
        "amp_y": Parameter.Type.DOUBLE,
        "amp_z": Parameter.Type.DOUBLE,
        "period_s": Parameter.Type.DOUBLE,
        "trajectory_ramp_s": Parameter.Type.DOUBLE,
        "roll_amplitude_deg": Parameter.Type.DOUBLE,
        "pitch_amplitude_deg": Parameter.Type.DOUBLE,
        "yaw_amplitude_deg": Parameter.Type.DOUBLE,
        "attitude_period_s": Parameter.Type.DOUBLE,
        "step_amplitude": Parameter.Type.DOUBLE,
        "step_time_s": Parameter.Type.DOUBLE,
        "move_duration_s": Parameter.Type.DOUBLE,
        "manual_vertical_speed_mps": Parameter.Type.DOUBLE,
        "station_linear_input_gain_mps": Parameter.Type.DOUBLE,
        "station_yaw_input_gain_rps": Parameter.Type.DOUBLE,
    }
    SCENARIO_PARAMETER_DEFAULTS = {
        # Managed motion tasks are relative unless a task explicitly selects
        # an absolute map-frame target, as pose_hold does.
        "relative_to_initial_pose": True,
        # Keep altitude_hold deterministic even while an older managed task
        # document without center_z remains installed on the edge computer.
        "center_z": 0.9,
    }

    def __init__(self):
        super().__init__("tracking_experiment")
        self.declare_parameter(
            "task_config_dir", "src/robotcore_runtime/config/tasks"
        )
        self.declare_parameter("trajectory_node_name", "/trajectory_command")
        self.authority = None
        self.latest_trajectory_target = None
        self.trajectory_target_sequence = 0
        self.latest_pid_status = None
        self.pid_status_sequence = 0
        # Action execute callbacks are advanced by rclpy's executor, not by an
        # asyncio event loop.  Use a separate callback group for one-shot ROS
        # timers so the action can yield without blocking the executor or
        # raising ``RuntimeError: no running event loop`` on Humble.
        self.wait_callback_group = MutuallyExclusiveCallbackGroup()
        self.wait_timers = set()
        self.create_subscription(
            ControlAuthorityStatus,
            "/control/authority/status",
            self.on_authority,
            10,
        )
        self.create_subscription(
            TrajectoryTarget,
            "/runtime/trajectory_target",
            self.on_trajectory_target,
            10,
            callback_group=self.wait_callback_group,
        )
        self.create_subscription(
            PidStatus,
            "/control/pid/status",
            self.on_pid_status,
            10,
            callback_group=self.wait_callback_group,
        )
        self.authority_client = self.create_client(
            SetControlAuthority, "/control/authority/set"
        )
        self.run_start_client = self.create_client(StartRun, "/runtime/run/start")
        self.run_stop_client = self.create_client(StopRun, "/runtime/run/stop")
        self.reset_client = self.create_client(Trigger, "/runtime/trajectory/reset")
        self.trajectory_stop_client = self.create_client(
            Trigger, "/runtime/trajectory/stop"
        )
        self.validate_client = self.create_client(
            Trigger, "/runtime/trajectory/validate"
        )
        trajectory_node = str(self.get_parameter("trajectory_node_name").value).rstrip("/")
        self.parameter_client = self.create_client(
            SetParameters, f"{trajectory_node}/set_parameters"
        )
        self.server = ActionServer(
            self,
            RunTrackingExperiment,
            "/runtime/run_tracking_experiment",
            self.execute,
            cancel_callback=lambda _goal: CancelResponse.ACCEPT,
        )

    def task_path(self, task_name):
        filename = f"{task_name}.json"
        path = Path(str(self.get_parameter("task_config_dir").value)) / filename
        if not path.is_file():
            path = (
                Path(get_package_share_directory("robotcore_runtime"))
                / "config"
                / "tasks"
                / filename
            )
        return path

    def load_task(self, task_name):
        return json.loads(self.task_path(task_name).read_text(encoding="utf-8"))

    def on_authority(self, message):
        self.authority = message

    def on_trajectory_target(self, message):
        self.latest_trajectory_target = message
        self.trajectory_target_sequence += 1

    def on_pid_status(self, message):
        self.latest_pid_status = message
        self.pid_status_sequence += 1

    async def execute(self, goal_handle):
        request = goal_handle.request
        task_name = str(request.scenario)
        if task_name not in {
            "pose_hold",
            "station_hold",
            "station_hold_fast",
            "spatial_figure_eight",
        }:
            return self.finish(
                goal_handle, False, "tracking strategy is not approved"
            )
        task = self.load_task(task_name)
        scenario = dict(task["trajectory"])
        run_until_stopped = bool(task.get("run_until_stopped", False))
        if str(request.controller) != task["controller"] or str(request.controller) != "pid":
            return self.finish(goal_handle, False, "only the pid controller is approved")
        if (
            self.authority is None
            or not self.authority.armed
            or self.authority.selected_source != "pid"
            or self.authority.fault_latched
        ):
            return self.finish(
                goal_handle, False, "PID must be selected, healthy, and explicitly armed"
            )

        duration = float(request.duration_s)
        if duration <= 0.0:
            duration = float(task["duration_s"])
        if not self.run_start_client.wait_for_service(timeout_sec=2.0):
            await self.disarm()
            return self.finish(goal_handle, False, "run recorder service unavailable")
        start_request = StartRun.Request()
        start_request.task_name = str(task["name"])
        start_request.controller = str(task["controller"])
        start_request.duration_s = duration
        start_result = await self.run_start_client.call_async(start_request)
        if not start_result.success:
            await self.disarm()
            return self.finish(goal_handle, False, start_result.message)
        run_dir = start_result.run_dir

        # Parameter updates and full-envelope validation are synchronous
        # callbacks in trajectory_command.  They can briefly pause its target
        # timer, so never leave automatic authority armed while preparing a
        # new task.  Re-arm only after reset and a fresh PID publication.
        disarm_result = await self.set_authority(False)
        if disarm_result is None or not disarm_result.accepted:
            return await self.fail_active_run(
                goal_handle, run_dir, "could not disarm PID while preparing trajectory"
            )

        if not self.parameter_client.wait_for_service(timeout_sec=2.0):
            return await self.fail_active_run(
                goal_handle, run_dir, "trajectory parameter service unavailable"
            )

        scenario_parameters = dict(self.SCENARIO_PARAMETER_DEFAULTS)
        scenario_parameters.update(scenario)
        parameters = []
        for name, parameter_type in self.ALLOWED_PARAMETER_TYPES.items():
            if name in scenario_parameters:
                parameters.append(
                    Parameter(name, parameter_type, scenario_parameters[name])
                )
        parameter_request = SetParameters.Request()
        parameter_request.parameters = [item.to_parameter_msg() for item in parameters]
        parameter_response = await self.parameter_client.call_async(parameter_request)
        if not all(result.successful for result in parameter_response.results):
            return await self.fail_active_run(
                goal_handle, run_dir, "trajectory scenario parameters rejected"
            )
        if not self.validate_client.wait_for_service(timeout_sec=2.0):
            return await self.fail_active_run(
                goal_handle, run_dir, "trajectory validation service unavailable"
            )
        validation_result = await self.validate_client.call_async(Trigger.Request())
        if not validation_result.success:
            return await self.fail_active_run(
                goal_handle, run_dir, validation_result.message
            )
        if not self.reset_client.wait_for_service(timeout_sec=2.0):
            return await self.fail_active_run(
                goal_handle, run_dir, "trajectory reset service unavailable"
            )
        target_sequence_before_reset = self.trajectory_target_sequence
        reset_result = await self.reset_client.call_async(Trigger.Request())
        if not reset_result.success:
            return await self.fail_active_run(
                goal_handle, run_dir, reset_result.message
            )
        if not await self.wait_for_tracking_ready(
            target_sequence_before_reset,
            str(scenario_parameters["trajectory_type"]),
            1.0,
        ):
            return await self.fail_active_run(
                goal_handle,
                run_dir,
                "reset trajectory target and ready PID command were not observed",
            )
        arm_result = await self.set_authority(True)
        if arm_result is None or not arm_result.accepted or not arm_result.armed:
            arm_message = (
                str(arm_result.message)
                if arm_result is not None
                else "control authority service unavailable"
            )
            return await self.fail_active_run(
                goal_handle, run_dir, f"PID re-arm after trajectory reset failed: {arm_message}"
            )
        if not await self.wait_for_armed_status(0.5):
            return await self.fail_active_run(
                goal_handle, run_dir, "PID re-arm was not confirmed by authority status"
            )
        # Readiness was established with the same zero-time approach target.
        # Reset the trajectory clock once more so the full 15-second smooth
        # center approach begins after authority is active, not during the
        # disarmed readiness handshake.
        official_start_result = await self.reset_client.call_async(Trigger.Request())
        if not official_start_result.success:
            return await self.fail_active_run(
                goal_handle, run_dir, official_start_result.message
            )
        started = self.get_clock().now().nanoseconds
        success = True
        canceled = False
        message = "completed"
        try:
            while True:
                elapsed = (self.get_clock().now().nanoseconds - started) * 1e-9
                if not run_until_stopped and elapsed >= duration:
                    break
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    success = False
                    canceled = True
                    message = "canceled and disarmed"
                    break
                if (
                    self.authority is None
                    or not self.authority.armed
                    or self.authority.fault_latched
                ):
                    success = False
                    message = "authority disarmed or faulted during experiment"
                    break
                feedback = RunTrackingExperiment.Feedback()
                feedback.progress = (
                    0.0
                    if run_until_stopped
                    else float(min(1.0, elapsed / max(duration, 1e-6)))
                )
                feedback.phase = (
                    "hold"
                    if elapsed < float(scenario.get("hold_before_motion_s", 0.0))
                    else "tracking"
                )
                feedback.elapsed_s = float(elapsed)
                goal_handle.publish_feedback(feedback)
                await self.wait_for_next_feedback(0.1)
        finally:
            try:
                await self.stop_trajectory()
            finally:
                await self.disarm()
        await self.stop_run(success, message)
        if success:
            goal_handle.succeed()
        elif not canceled:
            goal_handle.abort()
        result = RunTrackingExperiment.Result()
        result.success = success
        result.run_dir = run_dir
        result.message = message
        return result

    async def wait_for_next_feedback(self, delay_s):
        """Yield an action callback until a one-shot ROS timer fires."""

        future = Future()
        timer_holder = {}

        def wake():
            timer = timer_holder.get("timer")
            if timer is not None:
                timer.cancel()
                self.destroy_timer(timer)
                self.wait_timers.discard(timer)
            if not future.done():
                future.set_result(None)

        timer = self.create_timer(
            max(float(delay_s), 0.001),
            wake,
            callback_group=self.wait_callback_group,
        )
        timer_holder["timer"] = timer
        self.wait_timers.add(timer)
        await future

    async def fail_active_run(self, goal_handle, run_dir, message):
        try:
            await self.stop_trajectory()
        finally:
            await self.disarm()
        await self.stop_run(False, message)
        result = self.finish(goal_handle, False, message)
        result.run_dir = run_dir
        return result

    async def stop_run(self, success, message):
        request = StopRun.Request()
        request.success = bool(success)
        request.message = str(message)
        return await self.run_stop_client.call_async(request)

    async def stop_trajectory(self):
        if not self.trajectory_stop_client.wait_for_service(timeout_sec=1.0):
            return None
        return await self.trajectory_stop_client.call_async(Trigger.Request())

    async def set_authority(self, arm):
        if not self.authority_client.wait_for_service(timeout_sec=1.0):
            return None
        request = SetControlAuthority.Request()
        request.source = "pid"
        request.arm = bool(arm)
        request.clear_fault = False
        request.update_pwm_limit = False
        request.pwm_limit_us = 0.0
        return await self.authority_client.call_async(request)

    async def wait_for_tracking_ready(
        self, target_sequence_before_reset, trajectory_type, timeout_s
    ):
        """Wait for a reset target and then a PID status produced from it."""

        deadline_ns = self.get_clock().now().nanoseconds + int(timeout_s * 1e9)
        pid_sequence_at_target = None
        expected_type = str(trajectory_type).lower()
        while self.get_clock().now().nanoseconds < deadline_ns:
            target = self.latest_trajectory_target
            if (
                pid_sequence_at_target is None
                and self.trajectory_target_sequence > target_sequence_before_reset
                and target is not None
                and target.valid
                and str(target.trajectory_type).lower() == expected_type
            ):
                pid_sequence_at_target = self.pid_status_sequence
            pid = self.latest_pid_status
            if (
                pid_sequence_at_target is not None
                and self.pid_status_sequence > pid_sequence_at_target
                and pid is not None
                and pid.ready
                and pid.producing_command
                and not pid.missing_inputs
            ):
                return True
            await self.wait_for_next_feedback(0.02)
        return False

    async def wait_for_armed_status(self, timeout_s):
        deadline_ns = self.get_clock().now().nanoseconds + int(timeout_s * 1e9)
        while self.get_clock().now().nanoseconds < deadline_ns:
            if (
                self.authority is not None
                and self.authority.armed
                and not self.authority.fault_latched
                and self.authority.selected_source == "pid"
            ):
                return True
            await self.wait_for_next_feedback(0.02)
        return False

    async def disarm(self):
        return await self.set_authority(False)

    def finish(self, goal_handle, success, message):
        if success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        result = RunTrackingExperiment.Result()
        result.success = bool(success)
        result.run_dir = ""
        result.message = str(message)
        return result


def main(args=None):
    rclpy.init(args=args)
    node = TrackingExperimentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
