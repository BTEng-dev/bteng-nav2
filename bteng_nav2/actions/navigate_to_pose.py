"""NavigateToPose action node for Nav2."""

from __future__ import annotations

from bteng import InputPort, OutputPort
from bteng_ros2 import RosActionNode

from bteng_nav2.actions._result import action_result_of


class NavigateToPoseNode(RosActionNode):
    """Send a NavigateToPose goal to the Nav2 stack."""

    action_name = "/navigate_to_pose"

    @classmethod
    def provided_ports(cls):
        return [
            InputPort("goal_pose"),
            InputPort("behavior_tree", default=""),
            OutputPort("result"),
        ]

    def on_start(self):
        from nav2_msgs.action import NavigateToPose

        self.action_type = NavigateToPose
        return super().on_start()

    def make_goal(self):
        from nav2_msgs.action import NavigateToPose

        goal = NavigateToPose.Goal()
        goal.pose = self.get_input("goal_pose")
        goal.behavior_tree = self.get_input("behavior_tree", "")
        return goal

    def on_success(self):
        self.set_output("result", action_result_of(self))
