"""Goal-related publisher nodes for nav2 behavior trees."""

from bteng import ActionNode, InputPort
from bteng_ros2 import RosTopicMixin

from bteng_nav2.publishers._wait_for_subscriber import WaitForSubscriberMixin


class PublishGoalPoseNode(WaitForSubscriberMixin, RosTopicMixin, ActionNode):
    """Publish a PoseStamped to /goal_pose and return SUCCESS."""

    topic = "/goal_pose"

    @classmethod
    def provided_ports(cls):
        return [InputPort("goal_pose")]

    def tick(self):
        from geometry_msgs.msg import PoseStamped

        return self._publish_when_matched(PoseStamped, self.get_input("goal_pose"))
