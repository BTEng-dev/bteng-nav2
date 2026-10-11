"""FollowPath action node for Nav2."""

from __future__ import annotations

from bteng import InputPort
from bteng_ros2 import RosActionNode


class FollowPathNode(RosActionNode):
    """Send a FollowPath goal to the Nav2 stack."""

    action_name = "/follow_path"

    @classmethod
    def provided_ports(cls):
        return [
            InputPort("path"),
            InputPort("controller_id", default=""),
            InputPort("goal_checker_id", default=""),
        ]

    def on_start(self):
        from nav2_msgs.action import FollowPath

        self.action_type = FollowPath
        return super().on_start()

    def make_goal(self):
        from nav2_msgs.action import FollowPath

        goal = FollowPath.Goal()
        goal.path = self.get_input("path")
        goal.controller_id = self.get_input("controller_id", "")
        goal.goal_checker_id = self.get_input("goal_checker_id", "")
        return goal
