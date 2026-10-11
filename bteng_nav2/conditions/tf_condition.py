"""TF2-based condition nodes for nav2 behavior trees."""

import math

from bteng import ConditionNode, InputPort, NodeStatus
from bteng_ros2 import RosNodeMixin


class IsRobotAtFrameCondition(RosNodeMixin, ConditionNode):
    """Return SUCCESS when the robot (base_link) is within *tolerance* metres of *target_frame*."""

    @classmethod
    def provided_ports(cls):
        return [
            InputPort("target_frame"),
            InputPort("robot_frame", "base_link"),
            InputPort("tolerance", "Metres of slack when comparing the two frames", default=0.5),
        ]

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._tf_buffer = None
        self._tf_listener = None

    def tick(self):
        if self._tf_buffer is None:
            import rclpy.time  # noqa: F401 — ensures the module is available
            import tf2_ros

            self._tf_buffer = tf2_ros.Buffer()
            self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self._require_ros_node())

        target_frame = self.get_input("target_frame")
        robot_frame = self.get_input("robot_frame", "base_link")
        tolerance = float(self.get_input("tolerance", 0.5))

        try:
            import rclpy.time

            t = self._tf_buffer.lookup_transform(target_frame, robot_frame, rclpy.time.Time())
            dx = t.transform.translation.x
            dy = t.transform.translation.y
            dist = math.sqrt(dx * dx + dy * dy)
            return NodeStatus.SUCCESS if dist <= tolerance else NodeStatus.FAILURE
        except Exception:
            return NodeStatus.FAILURE
