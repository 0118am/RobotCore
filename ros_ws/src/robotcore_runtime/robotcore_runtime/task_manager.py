"""Minimal runtime task action server.

The action contract is present from Phase 1 so UI and mission code can integrate
before real task execution logic exists.
"""

import asyncio

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from robotcore_interfaces.action import RunTask


class TaskManager(Node):
    """Minimal action server for long-running runtime tasks."""

    def __init__(self):
        super().__init__("task_manager")
        self.server = ActionServer(self, RunTask, "/runtime/run_task", self.execute)

    async def execute(self, goal_handle):
        self.get_logger().info(f"Task requested: {goal_handle.request.task_name}")
        # Publish deterministic progress so action clients can validate their
        # feedback handling without depending on mission-specific behavior.
        for index in range(10):
            feedback = RunTask.Feedback()
            feedback.progress = float(index + 1) / 10.0
            feedback.state = "running"
            feedback.active_nodes = ["task_manager"]
            goal_handle.publish_feedback(feedback)
            await asyncio.sleep(0.1)

        goal_handle.succeed()
        result = RunTask.Result()
        result.success = True
        result.message = "Task skeleton completed"
        return result


def main(args=None):
    rclpy.init(args=args)
    node = TaskManager()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
