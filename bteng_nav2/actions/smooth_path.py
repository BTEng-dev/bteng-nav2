"""SmoothPath action node for Nav2."""

from __future__ import annotations

from bteng import InputPort, OutputPort
from bteng_ros2 import RosActionNode

from bteng_nav2.actions._result import action_result_of


class SmoothPathNode(RosActionNode):
    """Send a SmoothPath goal to the Nav2 stack."""

    action_name = "/smooth_path"

    @classmethod
    def provided_ports(cls):
        return [
            InputPort("path"),
            InputPort("smoother_id", default=""),
            InputPort("max_smoothing_duration", default=10.0),
            OutputPort("smoothed_path"),
        ]

    def on_start(self):
        from nav2_msgs.action import SmoothPath

        self.action_type = SmoothPath
        return super().on_start()

    def make_goal(self):
        from nav2_msgs.action import SmoothPath

        goal = SmoothPath.Goal()
        goal.path = self.get_input("path")
        goal.smoother_id = self.get_input("smoother_id", "")
        goal.max_smoothing_duration.sec = int(self.get_input("max_smoothing_duration", 10.0))
        return goal

    def on_success(self):
        self.set_output("smoothed_path", action_result_of(self).path)
