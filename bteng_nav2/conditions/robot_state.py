"""Robot-state condition nodes for nav2 behavior trees."""

from bteng import InputPort
from bteng_ros2 import RosConditionNode


class IsBatteryLowCondition(RosConditionNode):
    """Return SUCCESS when the battery level is below *low_threshold*."""

    topic_name = "/battery_state"
    topic_type = None  # set lazily on first tick

    @classmethod
    def provided_ports(cls):
        return [InputPort("low_threshold", "Battery fraction below which the check succeeds",
                          default=0.20)]

    def tick(self):
        if self.topic_type is None:
            from sensor_msgs.msg import BatteryState

            self.topic_type = BatteryState
        return super().tick()

    def evaluate(self, msg) -> bool:
        threshold = float(self.get_input("low_threshold", 0.20))
        return msg.percentage < threshold


class IsEmergencyStopCondition(RosConditionNode):
    """Return SUCCESS when the emergency stop is active."""

    topic_name = "/emergency_stop"
    topic_type = None  # set lazily on first tick

    @classmethod
    def provided_ports(cls):
        return [InputPort("topic_name", "Topic carrying the e-stop state",
                          default="/emergency_stop")]

    def tick(self):
        if self.topic_type is None:
            from std_msgs.msg import Bool

            self.topic_type = Bool
        # The port was declared but never read: RosConditionNode subscribes to
        # the class attribute, so `topic_name="/estop"` in XML bound cleanly,
        # passed every structural check and did nothing. Apply it before the
        # base class creates the subscription.
        self.topic_name = self.get_input("topic_name", type(self).topic_name)
        return super().tick()

    def evaluate(self, msg) -> bool:
        return bool(msg.data)
