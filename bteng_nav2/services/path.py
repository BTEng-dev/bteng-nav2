"""Nav2 path service nodes."""

from __future__ import annotations

from bteng import InputPort, NodeStatus
from bteng_ros2 import RosServiceNode


class IsPathValidNode(RosServiceNode):
    """Check whether a given path is valid (collision-free)."""

    service_name = "/is_path_valid"

    @classmethod
    def provided_ports(cls):
        return [
            InputPort("path"),
        ]

    def on_start(self):
        from nav2_msgs.srv import IsPathValid

        self.service_type = IsPathValid
        self._is_valid: bool = False
        return super().on_start()

    def make_request(self):
        from nav2_msgs.srv import IsPathValid

        req = IsPathValid.Request()
        req.path = self.get_input("path")
        return req

    def on_response(self, response) -> None:
        self._is_valid = response.is_valid

    def on_running(self) -> NodeStatus:
        status = super().on_running()
        if status == NodeStatus.SUCCESS and not self._is_valid:
            return NodeStatus.FAILURE
        return status
