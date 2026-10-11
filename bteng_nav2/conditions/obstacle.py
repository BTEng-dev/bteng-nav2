"""Obstacle-related condition nodes for nav2 behavior trees."""

import time

from bteng import InputPort
from bteng_ros2 import RosConditionNode


class IsObstacleNearCondition(RosConditionNode):
    """Return SUCCESS when a laser scan reading is closer than *min_distance*."""

    topic_name = "/scan"
    topic_type = None  # set lazily on first tick

    @classmethod
    def provided_ports(cls):
        return [InputPort("min_distance", "Metres below which a scan return counts as an obstacle",
                          default=0.5)]

    def tick(self):
        if self.topic_type is None:
            from sensor_msgs.msg import LaserScan

            self.topic_type = LaserScan
        return super().tick()

    def evaluate(self, msg) -> bool:
        valid_ranges = [r for r in msg.ranges if r > 0]
        if not valid_ranges:
            return False
        min_distance = float(self.get_input("min_distance", 0.5))
        return min(valid_ranges) < min_distance


class IsStuckCondition(RosConditionNode):
    """Return SUCCESS when velocity is near zero for longer than *stuck_timeout* seconds."""

    topic_name = "/odom"
    topic_type = None  # set lazily on first tick

    @classmethod
    def provided_ports(cls):
        return [
            InputPort(
                "stuck_timeout",
                "Seconds of near-zero velocity before the robot counts as stuck",
                default=3.0,
            )
        ]

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._stuck_since: float | None = None

    def tick(self):
        if self.topic_type is None:
            from nav_msgs.msg import Odometry

            self.topic_type = Odometry
        return super().tick()

    def evaluate(self, msg) -> bool:
        vx = msg.twist.twist.linear.x
        wz = msg.twist.twist.angular.z

        if abs(vx) < 0.01 and abs(wz) < 0.01:
            if self._stuck_since is None:
                self._stuck_since = time.monotonic()
            stuck_timeout = float(self.get_input("stuck_timeout", 3.0))
            return (time.monotonic() - self._stuck_since) > stuck_timeout
        else:
            self._stuck_since = None
            return False
