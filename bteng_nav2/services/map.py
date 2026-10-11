"""Nav2 map service nodes."""

from __future__ import annotations

from bteng import InputPort, OutputPort
from bteng_ros2 import RosServiceNode


class LoadMapNode(RosServiceNode):
    """Load a map from a YAML file via the map_server."""

    service_name = "/map_server/load_map"

    @classmethod
    def provided_ports(cls):
        return [
            InputPort("map_url"),
            OutputPort("result_code"),
        ]

    def on_start(self):
        from nav2_msgs.srv import LoadMap

        self.service_type = LoadMap
        return super().on_start()

    def make_request(self):
        from nav2_msgs.srv import LoadMap

        req = LoadMap.Request()
        req.map_url = self.get_input("map_url")
        return req

    def on_response(self, response) -> None:
        self.set_output("result_code", response.result)


class SaveMapNode(RosServiceNode):
    """Save the current map to disk via the map_saver service."""

    service_name = "/map_saver/save_map"

    @classmethod
    def provided_ports(cls):
        return [
            InputPort("map_url"),
            InputPort("map_topic", default="/map"),
            InputPort("free_thresh", default=0.25),
            InputPort("occupied_thresh", default=0.65),
            InputPort("image_format", default="png"),
            InputPort("map_mode", default="trinary"),
            OutputPort("result"),
        ]

    def on_start(self):
        from nav2_msgs.srv import SaveMap

        self.service_type = SaveMap
        return super().on_start()

    def make_request(self):
        from nav2_msgs.srv import SaveMap

        req = SaveMap.Request()
        req.map_url = self.get_input("map_url")
        req.map_topic = self.get_input("map_topic", "/map")
        req.free_thresh = self.get_input("free_thresh", 0.25)
        req.occupied_thresh = self.get_input("occupied_thresh", 0.65)
        req.image_format = self.get_input("image_format", "png")
        req.map_mode = self.get_input("map_mode", "trinary")
        return req

    def on_response(self, response) -> None:
        self.set_output("result", response.result)
