"""Recovery action nodes for Nav2."""

from __future__ import annotations

from bteng import InputPort
from bteng_ros2 import RosActionNode


class SpinNode(RosActionNode):
    """Send a Spin recovery goal to the Nav2 stack."""

    action_name = "/spin"

    @classmethod
    def provided_ports(cls):
        return [
            InputPort("target_yaw", "Rotation in radians", type_hint=float),
            InputPort("time_allowance", default=10.0),
        ]

    def on_start(self):
        from nav2_msgs.action import Spin

        self.action_type = Spin
        return super().on_start()

    def make_goal(self):
        from nav2_msgs.action import Spin

        goal = Spin.Goal()
        goal.target_yaw = self.get_input("target_yaw")
        goal.time_allowance.sec = int(self.get_input("time_allowance", 10.0))
        return goal


class BackUpNode(RosActionNode):
    """Send a BackUp recovery goal to the Nav2 stack."""

    action_name = "/backup"

    @classmethod
    def provided_ports(cls):
        return [
            InputPort("backup_dist", default=0.15),
            InputPort("backup_speed", default=0.025),
            InputPort("time_allowance", default=10.0),
        ]

    def on_start(self):
        from nav2_msgs.action import BackUp

        self.action_type = BackUp
        return super().on_start()

    def make_goal(self):
        from nav2_msgs.action import BackUp

        goal = BackUp.Goal()
        goal.target.x = self.get_input("backup_dist", 0.15)
        goal.speed = self.get_input("backup_speed", 0.025)
        goal.time_allowance.sec = int(self.get_input("time_allowance", 10.0))
        return goal


class DriveOnHeadingNode(RosActionNode):
    """Send a DriveOnHeading goal to the Nav2 stack."""

    action_name = "/drive_on_heading"

    @classmethod
    def provided_ports(cls):
        return [
            InputPort("dist_to_travel", "Distance in metres", type_hint=float),
            InputPort("speed", "Forward speed in m/s", type_hint=float),
            InputPort("time_allowance", default=10.0),
        ]

    def on_start(self):
        from nav2_msgs.action import DriveOnHeading

        self.action_type = DriveOnHeading
        return super().on_start()

    def make_goal(self):
        from nav2_msgs.action import DriveOnHeading

        goal = DriveOnHeading.Goal()
        goal.target.x = self.get_input("dist_to_travel")
        goal.speed = self.get_input("speed")
        goal.time_allowance.sec = int(self.get_input("time_allowance", 10.0))
        return goal


class WaitNode(RosActionNode):
    """Send a Wait recovery goal to the Nav2 stack."""

    action_name = "/wait"

    @classmethod
    def provided_ports(cls):
        return [
            InputPort("wait_duration", default=1),
        ]

    def on_start(self):
        from nav2_msgs.action import Wait

        self.action_type = Wait
        return super().on_start()

    def make_goal(self):
        from nav2_msgs.action import Wait

        goal = Wait.Goal()
        goal.time.sec = int(self.get_input("wait_duration", 1))
        return goal


class AssistedTeleopNode(RosActionNode):
    """Send an AssistedTeleop goal to the Nav2 stack."""

    action_name = "/assisted_teleop"

    @classmethod
    def provided_ports(cls):
        return [
            InputPort("time_allowance", default=30.0),
        ]

    def on_start(self):
        from nav2_msgs.action import AssistedTeleop

        self.action_type = AssistedTeleop
        return super().on_start()

    def make_goal(self):
        from nav2_msgs.action import AssistedTeleop

        goal = AssistedTeleop.Goal()
        goal.time_allowance.sec = int(self.get_input("time_allowance", 30.0))
        return goal
