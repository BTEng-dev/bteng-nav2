"""NavigateThroughPoses action node for Nav2."""

from __future__ import annotations

from bteng import InputPort, OutputPort
from bteng_ros2 import RosActionNode

from bteng_nav2.actions._result import action_result_of


class NavigateThroughPosesNode(RosActionNode):
    """Send a NavigateThroughPoses goal to the Nav2 stack."""

    action_name = "/navigate_through_poses"

    @classmethod
    def provided_ports(cls):
        return [
            InputPort("poses"),
            InputPort("behavior_tree", default=""),
            OutputPort("result"),
        ]

    def on_start(self):
        from nav2_msgs.action import NavigateThroughPoses

        self.action_type = NavigateThroughPoses
        return super().on_start()

    def make_goal(self):
        from nav2_msgs.action import NavigateThroughPoses

        goal = NavigateThroughPoses.Goal()
        goal.poses = self.get_input("poses")
        goal.behavior_tree = self.get_input("behavior_tree", "")
        return goal

    def on_success(self):
        self.set_output("result", action_result_of(self))
