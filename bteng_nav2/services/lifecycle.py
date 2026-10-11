"""Nav2 lifecycle manager service node."""

from __future__ import annotations

from bteng import InputPort, OutputPort
from bteng_ros2 import RosServiceNode

# Lifecycle command constants (mirrors ManageLifecycleNodes.Request constants)
LIFECYCLE_STARTUP = 0
LIFECYCLE_PAUSE = 1
LIFECYCLE_RESUME = 2
LIFECYCLE_RESET = 3
LIFECYCLE_SHUTDOWN = 4
LIFECYCLE_DESTROY = 5


class ManageLifecycleNodesNode(RosServiceNode):
    """Send a lifecycle management command to a Nav2 lifecycle manager."""

    service_name = "/lifecycle_manager_navigation/manage_nodes"

    @classmethod
    def provided_ports(cls):
        return [
            InputPort("command", default=0),
            InputPort("service_name", default="/lifecycle_manager_navigation/manage_nodes"),
            OutputPort("success"),
        ]

    def on_start(self):
        from nav2_msgs.srv import ManageLifecycleNodes

        self.service_type = ManageLifecycleNodes
        self.service_name = self.get_input(
            "service_name", "/lifecycle_manager_navigation/manage_nodes"
        )
        return super().on_start()

    def make_request(self):
        from nav2_msgs.srv import ManageLifecycleNodes

        req = ManageLifecycleNodes.Request()
        req.command = self.get_input("command", 0)
        return req

    def on_response(self, response) -> None:
        self.set_output("success", response.success)
