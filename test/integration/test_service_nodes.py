"""Integration tests for bteng_nav2 service nodes using FakeRosNode.

FakeRosNode.create_client() returns _FakeServiceClient, whose call_async()
fires callbacks synchronously (immediately). We use PreConfiguredFakeRosNode
to pre-load each service response before on_start() calls call_service().
"""
from __future__ import annotations

from unittest.mock import MagicMock

from bteng import Blackboard, NodeConfig, NodeStatus
from bteng_ros2.testing import FakeRosNode

# ---------------------------------------------------------------------------
# PreConfiguredFakeRosNode
# ---------------------------------------------------------------------------

class PreConfiguredFakeRosNode(FakeRosNode):
    """FakeRosNode that pre-loads service responses before create_client returns.

    Pass a dict mapping service names to pre-built response objects::

        fake = PreConfiguredFakeRosNode({"/my_srv": MagicMock()})
        node = MyServiceNode("test", config=config, ros_node=fake)
        node.on_start()   # creates client → response already loaded → callback fires
    """

    def __init__(self, responses: dict | None = None) -> None:
        super().__init__()
        self._responses: dict = responses or {}

    def create_client(self, srv_type, srv_name: str):
        client = super().create_client(srv_type, srv_name)
        if srv_name in self._responses:
            client.set_response(self._responses[srv_name])
        return client


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_config(**bb_data):
    bb = Blackboard()
    for k, v in bb_data.items():
        bb.set(k, v)
    return bb, NodeConfig(blackboard=bb, input_ports={k: k for k in bb_data})


# ---------------------------------------------------------------------------
# ClearGlobalCostmapNode
# ---------------------------------------------------------------------------

class TestClearGlobalCostmapIntegration:
    """End-to-end tests — real RosServiceClientMixin + FakeRosNode."""

    def test_clear_global_costmap_succeeds(self):
        """on_start() + on_running() → SUCCESS when service returns a response."""
        from nav2_msgs.srv import ClearEntireCostmap

        from bteng_nav2.services.costmap import ClearGlobalCostmapNode

        response = ClearEntireCostmap.Response()
        fake = PreConfiguredFakeRosNode({"/global_costmap/clear_entirely_global_costmap": response})
        bb, config = _make_config()
        node = ClearGlobalCostmapNode("test", config=config, ros_node=fake)

        start_status = node.on_start()
        assert start_status == NodeStatus.RUNNING

        run_status = node.on_running()
        assert run_status == NodeStatus.SUCCESS

    def test_clear_global_costmap_client_registered(self):
        """FakeRosNode registers the service client under the expected service name."""
        from nav2_msgs.srv import ClearEntireCostmap

        from bteng_nav2.services.costmap import ClearGlobalCostmapNode

        response = ClearEntireCostmap.Response()
        fake = PreConfiguredFakeRosNode({"/global_costmap/clear_entirely_global_costmap": response})
        bb, config = _make_config()
        node = ClearGlobalCostmapNode("test", config=config, ros_node=fake)

        node.on_start()

        assert "/global_costmap/clear_entirely_global_costmap" in fake.service_clients


# ---------------------------------------------------------------------------
# GetGlobalCostmapNode
# ---------------------------------------------------------------------------

class TestGetGlobalCostmapIntegration:
    """GetGlobalCostmapNode writes the received costmap to the output port."""

    def test_get_global_costmap_writes_output(self):
        from nav2_msgs.srv import GetCostmap

        from bteng_nav2.services.costmap import GetGlobalCostmapNode

        fake_map = MagicMock()
        response = GetCostmap.Response()
        response.map = fake_map

        fake = PreConfiguredFakeRosNode({"/global_costmap/get_costmap": response})
        bb = Blackboard()
        config = NodeConfig(
            blackboard=bb,
            input_ports={},
            output_ports={"costmap": "out_costmap"},
        )
        node = GetGlobalCostmapNode("test", config=config, ros_node=fake)

        node.on_start()
        status = node.on_running()

        assert status == NodeStatus.SUCCESS
        assert bb.get("out_costmap") is fake_map

    def test_get_global_costmap_returns_success(self):
        from nav2_msgs.srv import GetCostmap

        from bteng_nav2.services.costmap import GetGlobalCostmapNode

        response = GetCostmap.Response()
        fake = PreConfiguredFakeRosNode({"/global_costmap/get_costmap": response})
        bb, config = _make_config()
        node = GetGlobalCostmapNode("test", config=config, ros_node=fake)

        node.on_start()
        assert node.on_running() == NodeStatus.SUCCESS


# ---------------------------------------------------------------------------
# IsPathValidNode
# ---------------------------------------------------------------------------

class TestIsPathValidIntegration:
    """IsPathValidNode integration: checks the is_valid flag from the response."""

    def _make_node(self, is_valid: bool):
        from nav2_msgs.srv import IsPathValid

        from bteng_nav2.services.path import IsPathValidNode

        fake_path = MagicMock()
        response = IsPathValid.Response()
        response.is_valid = is_valid

        fake = PreConfiguredFakeRosNode({"/is_path_valid": response})
        bb, config = _make_config(path=fake_path)
        node = IsPathValidNode("test", config=config, ros_node=fake)
        return node

    def test_is_path_valid_returns_success(self):
        """is_valid=True → on_running() returns SUCCESS."""
        node = self._make_node(is_valid=True)
        node.on_start()
        assert node.on_running() == NodeStatus.SUCCESS

    def test_is_path_valid_returns_failure(self):
        """is_valid=False → on_running() returns FAILURE."""
        node = self._make_node(is_valid=False)
        node.on_start()
        assert node.on_running() == NodeStatus.FAILURE

    def test_is_path_valid_running_status_on_start(self):
        """on_start() always returns RUNNING (async service call in flight)."""
        node = self._make_node(is_valid=True)
        assert node.on_start() == NodeStatus.RUNNING


# ---------------------------------------------------------------------------
# LoadMapNode
# ---------------------------------------------------------------------------

class TestLoadMapIntegration:
    """LoadMapNode writes result_code to the blackboard output port."""

    def test_load_map_writes_result_code(self):
        from nav2_msgs.srv import LoadMap

        from bteng_nav2.services.map import LoadMapNode

        response = LoadMap.Response()
        response.result = 0  # RESULT_SUCCESS

        fake = PreConfiguredFakeRosNode({"/map_server/load_map": response})
        bb = Blackboard()
        bb.set("map_url", "/maps/test.yaml")
        config = NodeConfig(
            blackboard=bb,
            input_ports={"map_url": "map_url"},
            output_ports={"result_code": "load_result"},
        )
        node = LoadMapNode("test", config=config, ros_node=fake)

        node.on_start()
        status = node.on_running()

        assert status == NodeStatus.SUCCESS
        assert bb.get("load_result") == 0

    def test_load_map_returns_success(self):
        from nav2_msgs.srv import LoadMap

        from bteng_nav2.services.map import LoadMapNode

        response = LoadMap.Response()
        response.result = 0

        fake = PreConfiguredFakeRosNode({"/map_server/load_map": response})
        bb, config = _make_config(map_url="/maps/test.yaml")
        node = LoadMapNode("test", config=config, ros_node=fake)

        node.on_start()
        assert node.on_running() == NodeStatus.SUCCESS

    def test_load_map_on_start_returns_running(self):
        from nav2_msgs.srv import LoadMap

        from bteng_nav2.services.map import LoadMapNode

        response = LoadMap.Response()
        response.result = 1

        fake = PreConfiguredFakeRosNode({"/map_server/load_map": response})
        bb, config = _make_config(map_url="/maps/other.yaml")
        node = LoadMapNode("test", config=config, ros_node=fake)

        assert node.on_start() == NodeStatus.RUNNING

    def test_load_map_creates_service_client(self):
        """FakeRosNode registers the client under the correct service name."""
        from nav2_msgs.srv import LoadMap

        from bteng_nav2.services.map import LoadMapNode

        response = LoadMap.Response()
        fake = PreConfiguredFakeRosNode({"/map_server/load_map": response})
        bb, config = _make_config(map_url="/maps/test.yaml")
        node = LoadMapNode("test", config=config, ros_node=fake)

        node.on_start()

        assert "/map_server/load_map" in fake.service_clients
