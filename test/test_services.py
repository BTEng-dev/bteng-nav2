"""Tests for bteng_nav2 service nodes."""
from __future__ import annotations

from unittest.mock import MagicMock, patch

import pytest
from bteng import Blackboard, NodeConfig, NodeStatus

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def make_node(cls, **bb_data):
    bb = Blackboard()
    for k, v in bb_data.items():
        bb.set(k, v)
    config = NodeConfig(blackboard=bb, input_ports={k: k for k in bb_data})
    node = cls("test", config=config, ros_node=MagicMock())
    node._response_delivered = False
    return node


def _start_patched(node):
    """Context manager: patch init+call+ready so on_start() runs cleanly."""
    return (
        patch.object(node, "_init_service_client"),
        patch.object(node, "call_service"),
        patch.object(node, "service_is_ready", return_value=True),
    )


# ---------------------------------------------------------------------------
# ClearGlobalCostmapNode
# ---------------------------------------------------------------------------

class TestClearGlobalCostmapNode:
    @pytest.fixture(autouse=True)
    def _setup(self):
        from bteng_nav2.services.costmap import ClearGlobalCostmapNode
        self.node = make_node(ClearGlobalCostmapNode)

    def test_on_start_initializes_and_calls_service(self):
        p1, p2, p3 = _start_patched(self.node)
        with p1 as mock_init, p2 as mock_call, p3:
            result = self.node.on_start()
        mock_init.assert_called_once()
        mock_call.assert_called_once()
        assert result == NodeStatus.RUNNING

    def test_on_running_returns_running_while_in_flight(self):
        with patch.object(self.node, "service_status", return_value=NodeStatus.RUNNING):
            assert self.node.on_running() == NodeStatus.RUNNING

    def test_on_running_returns_success_when_done(self):
        with patch.object(self.node, "service_status", return_value=NodeStatus.SUCCESS), \
             patch.object(type(self.node), "service_response",
                          new_callable=lambda: property(lambda self: MagicMock())):
            assert self.node.on_running() == NodeStatus.SUCCESS


# ---------------------------------------------------------------------------
# ClearLocalCostmapNode
# ---------------------------------------------------------------------------

class TestClearLocalCostmapNode:
    @pytest.fixture(autouse=True)
    def _setup(self):
        from bteng_nav2.services.costmap import ClearLocalCostmapNode
        self.node = make_node(ClearLocalCostmapNode)

    def test_on_start_initializes_and_calls_service(self):
        p1, p2, p3 = _start_patched(self.node)
        with p1 as mock_init, p2 as mock_call, p3:
            result = self.node.on_start()
        mock_init.assert_called_once()
        mock_call.assert_called_once()
        assert result == NodeStatus.RUNNING

    def test_on_running_returns_running_while_in_flight(self):
        with patch.object(self.node, "service_status", return_value=NodeStatus.RUNNING):
            assert self.node.on_running() == NodeStatus.RUNNING

    def test_on_running_returns_success_when_done(self):
        with patch.object(self.node, "service_status", return_value=NodeStatus.SUCCESS), \
             patch.object(type(self.node), "service_response",
                          new_callable=lambda: property(lambda self: MagicMock())):
            assert self.node.on_running() == NodeStatus.SUCCESS


# ---------------------------------------------------------------------------
# ClearGlobalCostmapAroundRobotNode
# ---------------------------------------------------------------------------

class TestClearGlobalCostmapAroundRobotNode:
    @pytest.fixture(autouse=True)
    def _setup(self):
        from bteng_nav2.services.costmap import ClearGlobalCostmapAroundRobotNode
        self.node = make_node(ClearGlobalCostmapAroundRobotNode, reset_distance=2.5)

    def test_on_start_initializes_and_calls_service(self):
        p1, p2, p3 = _start_patched(self.node)
        with p1 as mock_init, p2 as mock_call, p3:
            result = self.node.on_start()
        mock_init.assert_called_once()
        mock_call.assert_called_once()
        assert result == NodeStatus.RUNNING

    def test_make_request_uses_reset_distance(self):
        req = self.node.make_request()
        assert req.reset_distance == 2.5

    def test_on_running_returns_running_while_in_flight(self):
        with patch.object(self.node, "service_status", return_value=NodeStatus.RUNNING):
            assert self.node.on_running() == NodeStatus.RUNNING

    def test_on_running_returns_success_when_done(self):
        with patch.object(self.node, "service_status", return_value=NodeStatus.SUCCESS), \
             patch.object(type(self.node), "service_response",
                          new_callable=lambda: property(lambda self: MagicMock())):
            assert self.node.on_running() == NodeStatus.SUCCESS


# ---------------------------------------------------------------------------
# ClearLocalCostmapAroundRobotNode
# ---------------------------------------------------------------------------

class TestClearLocalCostmapAroundRobotNode:
    @pytest.fixture(autouse=True)
    def _setup(self):
        from bteng_nav2.services.costmap import ClearLocalCostmapAroundRobotNode
        self.node = make_node(ClearLocalCostmapAroundRobotNode, reset_distance=0.5)

    def test_on_start_initializes_and_calls_service(self):
        p1, p2, p3 = _start_patched(self.node)
        with p1 as mock_init, p2 as mock_call, p3:
            result = self.node.on_start()
        mock_init.assert_called_once()
        mock_call.assert_called_once()
        assert result == NodeStatus.RUNNING

    def test_make_request_uses_reset_distance(self):
        req = self.node.make_request()
        assert req.reset_distance == 0.5

    def test_on_running_returns_running_while_in_flight(self):
        with patch.object(self.node, "service_status", return_value=NodeStatus.RUNNING):
            assert self.node.on_running() == NodeStatus.RUNNING

    def test_on_running_returns_success_when_done(self):
        with patch.object(self.node, "service_status", return_value=NodeStatus.SUCCESS), \
             patch.object(type(self.node), "service_response",
                          new_callable=lambda: property(lambda self: MagicMock())):
            assert self.node.on_running() == NodeStatus.SUCCESS


# ---------------------------------------------------------------------------
# GetGlobalCostmapNode
# ---------------------------------------------------------------------------

class TestGetGlobalCostmapNode:
    @pytest.fixture(autouse=True)
    def _setup(self):
        from bteng_nav2.services.costmap import GetGlobalCostmapNode
        self.node = make_node(GetGlobalCostmapNode)

    def test_on_start_initializes_and_calls_service(self):
        p1, p2, p3 = _start_patched(self.node)
        with p1 as mock_init, p2 as mock_call, p3:
            result = self.node.on_start()
        mock_init.assert_called_once()
        mock_call.assert_called_once()
        assert result == NodeStatus.RUNNING

    def test_on_running_returns_running_while_in_flight(self):
        with patch.object(self.node, "service_status", return_value=NodeStatus.RUNNING):
            assert self.node.on_running() == NodeStatus.RUNNING

    def test_on_running_sets_costmap_output_on_success(self):
        fake_map = MagicMock()
        fake_response = MagicMock()
        fake_response.map = fake_map
        with patch.object(self.node, "service_status", return_value=NodeStatus.SUCCESS), \
             patch.object(type(self.node), "service_response",
                          new_callable=lambda: property(lambda self: fake_response)), \
             patch.object(self.node, "set_output") as mock_set:
            status = self.node.on_running()
        assert status == NodeStatus.SUCCESS
        mock_set.assert_called_once_with("costmap", fake_map)


# ---------------------------------------------------------------------------
# GetLocalCostmapNode
# ---------------------------------------------------------------------------

class TestGetLocalCostmapNode:
    @pytest.fixture(autouse=True)
    def _setup(self):
        from bteng_nav2.services.costmap import GetLocalCostmapNode
        self.node = make_node(GetLocalCostmapNode)

    def test_on_start_initializes_and_calls_service(self):
        p1, p2, p3 = _start_patched(self.node)
        with p1 as mock_init, p2 as mock_call, p3:
            result = self.node.on_start()
        mock_init.assert_called_once()
        mock_call.assert_called_once()
        assert result == NodeStatus.RUNNING

    def test_on_running_returns_running_while_in_flight(self):
        with patch.object(self.node, "service_status", return_value=NodeStatus.RUNNING):
            assert self.node.on_running() == NodeStatus.RUNNING

    def test_on_running_sets_costmap_output_on_success(self):
        fake_map = MagicMock()
        fake_response = MagicMock()
        fake_response.map = fake_map
        with patch.object(self.node, "service_status", return_value=NodeStatus.SUCCESS), \
             patch.object(type(self.node), "service_response",
                          new_callable=lambda: property(lambda self: fake_response)), \
             patch.object(self.node, "set_output") as mock_set:
            status = self.node.on_running()
        assert status == NodeStatus.SUCCESS
        mock_set.assert_called_once_with("costmap", fake_map)


# ---------------------------------------------------------------------------
# IsPathValidNode
# ---------------------------------------------------------------------------

class TestIsPathValidNode:
    @pytest.fixture(autouse=True)
    def _setup(self):
        from bteng_nav2.services.path import IsPathValidNode
        fake_path = MagicMock()
        self.node = make_node(IsPathValidNode, path=fake_path)
        self.fake_path = fake_path

    def test_on_start_initializes_and_calls_service(self):
        p1, p2, p3 = _start_patched(self.node)
        with p1 as mock_init, p2 as mock_call, p3:
            result = self.node.on_start()
        mock_init.assert_called_once()
        mock_call.assert_called_once()
        assert result == NodeStatus.RUNNING

    def test_on_running_returns_running_while_in_flight(self):
        p1, p2, p3 = _start_patched(self.node)
        with p1, p2, p3:
            self.node.on_start()
        with patch.object(self.node, "service_status", return_value=NodeStatus.RUNNING):
            assert self.node.on_running() == NodeStatus.RUNNING

    def test_on_running_returns_success_when_path_is_valid(self):
        fake_response = MagicMock()
        fake_response.is_valid = True
        p1, p2, p3 = _start_patched(self.node)
        with p1, p2, p3:
            self.node.on_start()
        with patch.object(self.node, "service_status", return_value=NodeStatus.SUCCESS), \
             patch.object(type(self.node), "service_response",
                          new_callable=lambda: property(lambda self: fake_response)):
            assert self.node.on_running() == NodeStatus.SUCCESS

    def test_on_running_returns_failure_when_path_is_invalid(self):
        fake_response = MagicMock()
        fake_response.is_valid = False
        p1, p2, p3 = _start_patched(self.node)
        with p1, p2, p3:
            self.node.on_start()
        with patch.object(self.node, "service_status", return_value=NodeStatus.SUCCESS), \
             patch.object(type(self.node), "service_response",
                          new_callable=lambda: property(lambda self: fake_response)):
            assert self.node.on_running() == NodeStatus.FAILURE

    def test_make_request_sets_path(self):
        req = self.node.make_request()
        assert req.path == self.fake_path


# ---------------------------------------------------------------------------
# LoadMapNode
# ---------------------------------------------------------------------------

class TestLoadMapNode:
    @pytest.fixture(autouse=True)
    def _setup(self):
        from bteng_nav2.services.map import LoadMapNode
        self.node = make_node(LoadMapNode, map_url="/maps/my_map.yaml")

    def test_on_start_initializes_and_calls_service(self):
        p1, p2, p3 = _start_patched(self.node)
        with p1 as mock_init, p2 as mock_call, p3:
            result = self.node.on_start()
        mock_init.assert_called_once()
        mock_call.assert_called_once()
        assert result == NodeStatus.RUNNING

    def test_on_running_returns_running_while_in_flight(self):
        with patch.object(self.node, "service_status", return_value=NodeStatus.RUNNING):
            assert self.node.on_running() == NodeStatus.RUNNING

    def test_on_running_sets_result_code_output_on_success(self):
        fake_response = MagicMock()
        fake_response.result = 0
        with patch.object(self.node, "service_status", return_value=NodeStatus.SUCCESS), \
             patch.object(type(self.node), "service_response",
                          new_callable=lambda: property(lambda self: fake_response)), \
             patch.object(self.node, "set_output") as mock_set:
            status = self.node.on_running()
        assert status == NodeStatus.SUCCESS
        mock_set.assert_called_once_with("result_code", 0)

    def test_make_request_sets_map_url(self):
        req = self.node.make_request()
        assert req.map_url == "/maps/my_map.yaml"


class TestEndpointsMatchNav2:
    """Endpoint names verified against ros-navigation/navigation2 (main).

    These were wrong, and stub tests could never show it: the fakes answer to
    whatever name they are handed, so a node aimed at a server that does not
    exist passed every test here and would fail on the first real stack.

    Sources checked:
      nav2_costmap_2d/src/clear_costmap_service.cpp -- services are
        "clear_entirely_"/"clear_around_" + the costmap's name, advertised
        inside that costmap node's own namespace
      nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml
        -- uses global_costmap/clear_entirely_global_costmap
      nav2_bringup/params/nav2_params.yaml -- behavior_plugins names the reverse
        behaviour "backup", so the action server is /backup, not /back_up
    """

    EXPECTED = {
        "ClearGlobalCostmapNode": "/global_costmap/clear_entirely_global_costmap",
        "ClearLocalCostmapNode": "/local_costmap/clear_entirely_local_costmap",
        "ClearGlobalCostmapAroundRobotNode": "/global_costmap/clear_around_global_costmap",
        "ClearLocalCostmapAroundRobotNode": "/local_costmap/clear_around_local_costmap",
        "GetGlobalCostmapNode": "/global_costmap/get_costmap",
        "GetLocalCostmapNode": "/local_costmap/get_costmap",
        "IsPathValidNode": "/is_path_valid",
        "LoadMapNode": "/map_server/load_map",
        "SaveMapNode": "/map_saver/save_map",
        "ManageLifecycleNodesNode": "/lifecycle_manager_navigation/manage_nodes",
    }

    def test_service_endpoints(self):
        import bteng_nav2

        for class_name, endpoint in self.EXPECTED.items():
            cls = getattr(bteng_nav2, class_name)
            assert cls.service_name == endpoint, f"{class_name} points at {cls.service_name}"

    def test_backup_action_is_named_backup(self):
        from bteng_nav2 import BackUpNode

        assert BackUpNode.action_name == "/backup"

    def test_costmap_services_live_under_their_costmap_node(self):
        import bteng_nav2

        for class_name in self.EXPECTED:
            if "Costmap" not in class_name:
                continue
            name = getattr(bteng_nav2, class_name).service_name
            assert name.startswith(("/global_costmap/", "/local_costmap/")), name
