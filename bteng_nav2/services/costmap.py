"""Nav2 costmap service nodes."""

from __future__ import annotations

from bteng import InputPort, OutputPort
from bteng_ros2 import RosServiceNode


class ClearGlobalCostmapNode(RosServiceNode):
    """Clear the global costmap entirely."""

    service_name = "/global_costmap/clear_entirely_global_costmap"

    @classmethod
    def provided_ports(cls):
        return []

    def on_start(self):
        from nav2_msgs.srv import ClearEntireCostmap

        self.service_type = ClearEntireCostmap
        return super().on_start()

    def make_request(self):
        from nav2_msgs.srv import ClearEntireCostmap

        return ClearEntireCostmap.Request()


class ClearLocalCostmapNode(RosServiceNode):
    """Clear the local costmap entirely."""

    service_name = "/local_costmap/clear_entirely_local_costmap"

    @classmethod
    def provided_ports(cls):
        return []

    def on_start(self):
        from nav2_msgs.srv import ClearEntireCostmap

        self.service_type = ClearEntireCostmap
        return super().on_start()

    def make_request(self):
        from nav2_msgs.srv import ClearEntireCostmap

        return ClearEntireCostmap.Request()


class ClearGlobalCostmapAroundRobotNode(RosServiceNode):
    """Clear the global costmap within a given radius around the robot."""

    service_name = "/global_costmap/clear_around_global_costmap"

    @classmethod
    def provided_ports(cls):
        return [
            InputPort("reset_distance", default=1.0),
        ]

    def on_start(self):
        from nav2_msgs.srv import ClearCostmapAroundRobot

        self.service_type = ClearCostmapAroundRobot
        return super().on_start()

    def make_request(self):
        from nav2_msgs.srv import ClearCostmapAroundRobot

        req = ClearCostmapAroundRobot.Request()
        req.reset_distance = self.get_input("reset_distance", 1.0)
        return req


class ClearLocalCostmapAroundRobotNode(RosServiceNode):
    """Clear the local costmap within a given radius around the robot."""

    service_name = "/local_costmap/clear_around_local_costmap"

    @classmethod
    def provided_ports(cls):
        return [
            InputPort("reset_distance", default=1.0),
        ]

    def on_start(self):
        from nav2_msgs.srv import ClearCostmapAroundRobot

        self.service_type = ClearCostmapAroundRobot
        return super().on_start()

    def make_request(self):
        from nav2_msgs.srv import ClearCostmapAroundRobot

        req = ClearCostmapAroundRobot.Request()
        req.reset_distance = self.get_input("reset_distance", 1.0)
        return req


class GetGlobalCostmapNode(RosServiceNode):
    """Retrieve the current global costmap."""

    service_name = "/global_costmap/get_costmap"

    @classmethod
    def provided_ports(cls):
        return [
            OutputPort("costmap"),
        ]

    def on_start(self):
        from nav2_msgs.srv import GetCostmap

        self.service_type = GetCostmap
        return super().on_start()

    def make_request(self):
        from nav2_msgs.srv import GetCostmap

        return GetCostmap.Request()

    def on_response(self, response) -> None:
        self.set_output("costmap", response.map)


class GetLocalCostmapNode(RosServiceNode):
    """Retrieve the current local costmap."""

    service_name = "/local_costmap/get_costmap"

    @classmethod
    def provided_ports(cls):
        return [
            OutputPort("costmap"),
        ]

    def on_start(self):
        from nav2_msgs.srv import GetCostmap

        self.service_type = GetCostmap
        return super().on_start()

    def make_request(self):
        from nav2_msgs.srv import GetCostmap

        return GetCostmap.Request()

    def on_response(self, response) -> None:
        self.set_output("costmap", response.map)
