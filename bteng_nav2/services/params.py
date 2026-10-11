"""Nav2 costmap parameter service nodes."""

from __future__ import annotations

from bteng import InputPort, OutputPort
from bteng_ros2 import RosServiceNode


class SetCostmapParameterNode(RosServiceNode):
    """Dynamically set a costmap parameter on a running Nav2 costmap node."""

    service_name = ""  # set dynamically in on_start

    @classmethod
    def provided_ports(cls):
        return [
            InputPort("ros_node_name"),
            InputPort("param_name"),
            InputPort("param_value", "Parameter value (double)", type_hint=float),
        ]

    def on_start(self):
        from rcl_interfaces.srv import SetParameters

        node_name = self.get_input("ros_node_name")
        self.service_name = f"/{node_name}/set_parameters"
        self.service_type = SetParameters
        return super().on_start()

    def make_request(self):
        from rcl_interfaces.msg import Parameter, ParameterValue
        from rcl_interfaces.srv import SetParameters

        req = SetParameters.Request()
        pv = ParameterValue()
        pv.type = 3  # PARAMETER_DOUBLE
        pv.double_value = float(self.get_input("param_value"))
        p = Parameter()
        p.name = self.get_input("param_name")
        p.value = pv
        req.parameters = [p]
        return req

    def on_response(self, response) -> None:
        pass


class GetCostmapParameterNode(RosServiceNode):
    """Read a costmap parameter from a running Nav2 costmap node."""

    service_name = ""  # set dynamically in on_start

    @classmethod
    def provided_ports(cls):
        return [
            InputPort("ros_node_name"),
            InputPort("param_name"),
            OutputPort("param_value"),
        ]

    def on_start(self):
        from rcl_interfaces.srv import GetParameters

        node_name = self.get_input("ros_node_name")
        self.service_name = f"/{node_name}/get_parameters"
        self.service_type = GetParameters
        return super().on_start()

    def make_request(self):
        from rcl_interfaces.srv import GetParameters

        req = GetParameters.Request()
        req.names = [self.get_input("param_name")]
        return req

    def on_response(self, response) -> None:
        if response.values:
            self.set_output("param_value", response.values[0].double_value)
