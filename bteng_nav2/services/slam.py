"""SLAM Toolbox service nodes."""

from __future__ import annotations

from bteng import InputPort
from bteng_ros2 import RosServiceNode


class SaveSlamMapNode(RosServiceNode):
    """Save the current SLAM map to disk via slam_toolbox."""

    service_name = "/slam_toolbox/save_map"

    @classmethod
    def provided_ports(cls):
        return [InputPort("filename")]

    def on_start(self):
        from slam_toolbox.srv import SaveMap

        self.service_type = SaveMap
        return super().on_start()

    def make_request(self):
        from slam_toolbox.srv import SaveMap
        from std_msgs.msg import String

        req = SaveMap.Request()
        # slam_toolbox declares this field as std_msgs/String, not string: it
        # takes a *message*, and assigning a plain str makes rclpy abort the
        # process rather than raise -- found on the first live run.
        req.name = String(data=str(self.get_input("filename")))
        return req

    def on_response(self, response) -> None:
        pass


class SerializePoseGraphNode(RosServiceNode):
    """Serialize the SLAM pose graph via slam_toolbox."""

    service_name = "/slam_toolbox/serialize_map"

    @classmethod
    def provided_ports(cls):
        return [InputPort("filename")]

    def on_start(self):
        from slam_toolbox.srv import SerializePoseGraph

        self.service_type = SerializePoseGraph
        return super().on_start()

    def make_request(self):
        from slam_toolbox.srv import SerializePoseGraph

        req = SerializePoseGraph.Request()
        # Plain `string` here, unlike SaveMap.name above -- the two services in
        # the same package spell it differently.
        req.filename = str(self.get_input("filename"))
        return req

    def on_response(self, response) -> None:
        pass
