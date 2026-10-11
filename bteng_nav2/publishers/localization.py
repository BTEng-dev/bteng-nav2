"""Localization-related publisher nodes for nav2 behavior trees."""

from bteng import ActionNode, InputPort
from bteng_ros2 import RosTopicMixin

from bteng_nav2.publishers._wait_for_subscriber import WaitForSubscriberMixin


class SetInitialPoseNode(WaitForSubscriberMixin, RosTopicMixin, ActionNode):
    """Publish a PoseWithCovarianceStamped to /initialpose and return SUCCESS."""

    topic = "/initialpose"

    @classmethod
    def provided_ports(cls):
        return [InputPort("initial_pose")]

    def tick(self):
        from geometry_msgs.msg import PoseWithCovarianceStamped

        return self._publish_when_matched(PoseWithCovarianceStamped, self.get_input("initial_pose"))
