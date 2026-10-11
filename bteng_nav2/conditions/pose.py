"""Pose-related condition nodes for nav2 behavior trees."""

import math
import time

from bteng import InputPort
from bteng_ros2 import RosConditionNode


class IsRobotNearGoalCondition(RosConditionNode):
    """Return SUCCESS when the robot is within *tolerance* metres of *goal_pose*."""

    topic_name = "/amcl_pose"
    topic_type = None  # set lazily on first tick

    @classmethod
    def provided_ports(cls):
        return [
            InputPort("goal_pose"),
            InputPort("tolerance", "Metres from the goal that count as arrived", default=0.5),
        ]

    def tick(self):
        if self.topic_type is None:
            from geometry_msgs.msg import PoseWithCovarianceStamped

            self.topic_type = PoseWithCovarianceStamped
        return super().tick()

    def evaluate(self, msg) -> bool:
        robot_x = msg.pose.pose.position.x
        robot_y = msg.pose.pose.position.y

        goal = self.get_input("goal_pose")
        goal_x = goal.pose.position.x
        goal_y = goal.pose.position.y

        tolerance = self.get_input("tolerance", 0.5)
        distance = math.sqrt((robot_x - goal_x) ** 2 + (robot_y - goal_y) ** 2)
        return distance < float(tolerance)


class IsLocalizationActiveCondition(RosConditionNode):
    """Return SUCCESS when an AMCL pose has been received within *timeout* seconds."""

    topic_name = "/amcl_pose"
    topic_type = None  # set lazily on first tick

    @classmethod
    def provided_ports(cls):
        return [InputPort("timeout", "Seconds without a pose before localization counts as stale",
                          default=2.0)]

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._last_msg_time: float = float("-inf")

    def tick(self):
        if self.topic_type is None:
            from geometry_msgs.msg import PoseWithCovarianceStamped

            self.topic_type = PoseWithCovarianceStamped
        return super().tick()

    def _on_msg(self, msg):
        self._last_msg_time = time.monotonic()
        super()._on_msg(msg)

    def evaluate(self, msg) -> bool:
        timeout = float(self.get_input("timeout", 2.0))
        return (time.monotonic() - self._last_msg_time) < timeout
