"""Operator-armed, allow-listed tracking experiment action server."""

from __future__ import annotations

import json
from pathlib import Path

import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.task import Future
from rcl_interfaces.srv import SetParameters
from std_srvs.srv import Trigger

from robotcore_interfaces.action import RunTrackingExperiment
from robotcore_interfaces.msg import ControlAuthorityStatus
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
        "roll_amplitude_deg": Parameter.Type.DOUBLE,
        "pitch_amplitude_deg": Parameter.Type.DOUBLE,
        "yaw_amplitude_deg": Parameter.Type.DOUBLE,
        "attitude_period_s": Parameter.Type.DOUBLE,
        "step_amplitude": Parameter.Type.DOUBLE,
        "step_time_s": Parameter.Type.DOUBLE,
        "move_duration_s": Parameter.Type.DOUBLE,
        "manual_vertical_speed_mps": Parameter.Type.DOUBLE,
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

    def load_task(self, task_name):
        path = Path(str(self.get_parameter("task_config_dir").value)) / f"{task_name}.json"
        return json.loads(path.read_text(encoding="utf-8"))

    def on_authority(self, message):
        self.authority = message

    async def execute(self, goal_handle):
        request = goal_handle.request
        task = self.load_task(str(request.scenario))
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
        reset_result = await self.reset_client.call_async(Trigger.Request())
        if not reset_result.success:
            return await self.fail_active_run(
                goal_handle, run_dir, reset_result.message
            )
        started = self.get_clock().now().nanoseconds
        success = True
        message = "completed"
        try:
            while True:
                elapsed = (self.get_clock().now().nanoseconds - started) * 1e-9
                if not run_until_stopped and elapsed >= duration:
                    break
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    success = False
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
        elif not goal_handle.is_cancel_requested:
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

    async def disarm(self):
        if not self.authority_client.wait_for_service(timeout_sec=1.0):
            return
        request = SetControlAuthority.Request()
        request.source = "pid"
        request.arm = False
        request.clear_fault = False
        await self.authority_client.call_async(request)

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
