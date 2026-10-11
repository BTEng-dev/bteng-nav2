"""Navigation-status condition nodes for nav2 behavior trees."""

from bteng import ConditionNode, InputPort, NodeStatus
from bteng_ros2 import RosNodeMixin


class IsGoalUpdatedCondition(ConditionNode):
    """Return SUCCESS when the goal pose has changed since the last call."""

    HASH_KEY = "_bteng_nav2_goal_hash"

    @classmethod
    def provided_ports(cls):
        return [InputPort("goal_pose")]

    def tick(self):
        goal = self.get_input("goal_pose")
        current_hash = hash(str(goal))
        last_hash = self.blackboard.get(self.HASH_KEY) if self.blackboard else None
        if current_hash != last_hash:
            if self.blackboard:
                self.blackboard.set(self.HASH_KEY, current_hash)
            return NodeStatus.SUCCESS
        return NodeStatus.FAILURE


class IsNavigationActiveCondition(RosNodeMixin, ConditionNode):
    """Return SUCCESS when the NavigateToPose action server is ready."""

    SERVER = "/navigate_to_pose"

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._client = None

    def tick(self):
        if self._client is None:
            from nav2_msgs.action import NavigateToPose
            from rclpy.action import ActionClient

            self._client = ActionClient(self._require_ros_node(), NavigateToPose, self.SERVER)
        return NodeStatus.SUCCESS if self._client.server_is_ready() else NodeStatus.FAILURE
