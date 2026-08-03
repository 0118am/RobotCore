"""Operator-armed, allow-listed tracking experiment action server."""

from __future__ import annotations

import asyncio
import json
from pathlib import Path

import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters
from std_msgs.msg import String
from std_srvs.srv import Trigger
import yaml

from robotcore_interfaces.action import RunTrackingExperiment
from robotcore_interfaces.msg import ControlAuthorityStatus
from robotcore_interfaces.srv import SetControlAuthority


class TrackingExperimentNode(Node):
    """Configure a safe scenario, report progress, and always disarm at exit."""

    ALLOWED_PARAMETER_TYPES = {
        "trajectory_type": Parameter.Type.STRING,
        "attitude_mode": Parameter.Type.STRING,
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
    }

    def __init__(self):
        super().__init__("tracking_experiment")
        self.declare_parameter(
            "scenario_config_path", "src/robotcore_runtime/config/tracking_scenarios.yaml"
        )
        self.declare_parameter("trajectory_node_name", "/trajectory_command")
        self.scenarios = self.load_scenarios()
        self.authority = None
        self.run_dir = ""
        self.create_subscription(
            ControlAuthorityStatus,
            "/control/authority/status",
            self.on_authority,
            10,
        )
        self.create_subscription(String, "/runtime/run_dir", self.on_run_dir, 10)
        self.event_publisher = self.create_publisher(
            String, "/runtime/tracking_experiment/event", 10
        )
        self.authority_client = self.create_client(
            SetControlAuthority, "/control/authority/set"
        )
        self.reset_client = self.create_client(Trigger, "/runtime/trajectory/reset")
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

    def load_scenarios(self):
        path = Path(str(self.get_parameter("scenario_config_path").value))
        data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
        return dict(data.get("scenarios") or {})

    def on_authority(self, message):
        self.authority = message

    def on_run_dir(self, message):
        self.run_dir = str(message.data)

    async def execute(self, goal_handle):
        request = goal_handle.request
        scenario = self.scenarios.get(str(request.scenario))
        if scenario is None:
            return self.finish(goal_handle, False, f"unknown scenario: {request.scenario}")
        if str(request.controller) != "pid":
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
        if not self.parameter_client.wait_for_service(timeout_sec=2.0):
            await self.disarm()
            return self.finish(goal_handle, False, "trajectory parameter service unavailable")

        parameters = []
        for name, parameter_type in self.ALLOWED_PARAMETER_TYPES.items():
            if name in scenario:
                parameters.append(Parameter(name, parameter_type, scenario[name]))
        parameter_request = SetParameters.Request()
        parameter_request.parameters = [item.to_parameter_msg() for item in parameters]
        parameter_response = await self.parameter_client.call_async(parameter_request)
        if not all(result.successful for result in parameter_response.results):
            await self.disarm()
            return self.finish(goal_handle, False, "trajectory scenario parameters rejected")
        if not self.validate_client.wait_for_service(timeout_sec=2.0):
            await self.disarm()
            return self.finish(goal_handle, False, "trajectory validation service unavailable")
        validation_result = await self.validate_client.call_async(Trigger.Request())
        if not validation_result.success:
            await self.disarm()
            return self.finish(goal_handle, False, validation_result.message)
        if not self.reset_client.wait_for_service(timeout_sec=2.0):
            await self.disarm()
            return self.finish(goal_handle, False, "trajectory reset service unavailable")
        reset_result = await self.reset_client.call_async(Trigger.Request())
        if not reset_result.success:
            await self.disarm()
            return self.finish(goal_handle, False, reset_result.message)

        duration = float(request.duration_s)
        if duration <= 0.0:
            duration = float(scenario.get("default_duration_s", 30.0))
        self.publish_experiment_event(
            "start", str(request.scenario), duration, True, "started"
        )
        started = self.get_clock().now().nanoseconds
        success = True
        message = "completed"
        try:
            while True:
                elapsed = (self.get_clock().now().nanoseconds - started) * 1e-9
                if elapsed >= duration:
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
                feedback.progress = float(min(1.0, elapsed / duration))
                feedback.phase = (
                    "hold"
                    if elapsed < float(scenario.get("hold_before_motion_s", 0.0))
                    else "tracking"
                )
                feedback.elapsed_s = float(elapsed)
                goal_handle.publish_feedback(feedback)
                await asyncio.sleep(0.1)
        finally:
            await self.disarm()
        self.publish_experiment_event(
            "end", str(request.scenario), duration, success, message
        )
        if success:
            goal_handle.succeed()
        elif not goal_handle.is_cancel_requested:
            goal_handle.abort()
        result = RunTrackingExperiment.Result()
        result.success = success
        result.run_dir = self.run_dir
        result.message = message
        return result

    def publish_experiment_event(self, phase, scenario, duration, success, message):
        event = String()
        event.data = json.dumps(
            {
                "phase": str(phase),
                "scenario": str(scenario),
                "controller": "pid",
                "duration_s": float(duration),
                "success": bool(success),
                "message": str(message),
            },
            sort_keys=True,
        )
        self.event_publisher.publish(event)

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
        result.run_dir = self.run_dir
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
