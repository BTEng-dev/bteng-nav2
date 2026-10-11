"""Tests for ManageLifecycleNodesNode and SaveMapNode."""
from __future__ import annotations

import sys
import types as _types
from unittest.mock import MagicMock, patch

import pytest
from bteng import Blackboard, NodeConfig, NodeStatus

# ---------------------------------------------------------------------------
# Stubs: add ManageLifecycleNodes and SaveMap to nav2_msgs.srv
# ---------------------------------------------------------------------------

_nav2_srv = sys.modules.get("nav2_msgs.srv", _types.ModuleType("nav2_msgs.srv"))


class _ManageLifecycleNodes:
    STARTUP = 0
    PAUSE = 1
    RESUME = 2
    RESET = 3
    SHUTDOWN = 4
    DESTROY = 5

    class Request:
        def __init__(self):
            self.command = 0

    class Response:
        def __init__(self):
            self.success = True


class _SaveMap:
    class Request:
        def __init__(self):
            self.map_topic = "/map"
            self.map_url = ""
            self.free_thresh = 0.25
            self.occupied_thresh = 0.65
            self.image_format = "png"
            self.map_mode = "trinary"

    class Response:
        def __init__(self):
            self.result = True


_nav2_srv.ManageLifecycleNodes = _ManageLifecycleNodes
_nav2_srv.SaveMap = _SaveMap
sys.modules["nav2_msgs.srv"] = _nav2_srv


# ---------------------------------------------------------------------------
# Helpers (same as test_services.py)
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
# ManageLifecycleNodesNode
# ---------------------------------------------------------------------------

class TestManageLifecycleNodesNode:
    @pytest.fixture(autouse=True)
    def _setup(self):
        from bteng_nav2.services.lifecycle import ManageLifecycleNodesNode
        self.node = make_node(ManageLifecycleNodesNode)

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
        fake_response = MagicMock()
        fake_response.success = True
        with patch.object(self.node, "service_status", return_value=NodeStatus.SUCCESS), \
             patch.object(type(self.node), "service_response",
                          new_callable=lambda: property(lambda self: fake_response)):
            assert self.node.on_running() == NodeStatus.SUCCESS

    def test_make_request_sets_command(self):
        req = self.node.make_request()
        # default command is STARTUP (0)
        assert req.command == 0

    def test_default_command_is_startup(self):
        from bteng_nav2.services.lifecycle import LIFECYCLE_STARTUP
        p1, p2, p3 = _start_patched(self.node)
        with p1, p2, p3:
            self.node.on_start()
        req = self.node.make_request()
        assert req.command == LIFECYCLE_STARTUP

    def test_custom_service_name_is_used(self):
        from bteng_nav2.services.lifecycle import ManageLifecycleNodesNode
        custom_name = "/lifecycle_manager_localization/manage_nodes"
        node = make_node(ManageLifecycleNodesNode, service_name=custom_name)
        p1, p2, p3 = _start_patched(node)
        with p1, p2, p3:
            node.on_start()
        assert node.service_name == custom_name

    def test_on_response_sets_success_output(self):
        fake_response = MagicMock()
        fake_response.success = True
        with patch.object(self.node, "set_output") as mock_set:
            self.node.on_response(fake_response)
        mock_set.assert_called_once_with("success", True)

    def test_on_response_sets_success_false_on_failure(self):
        fake_response = MagicMock()
        fake_response.success = False
        with patch.object(self.node, "set_output") as mock_set:
            self.node.on_response(fake_response)
        mock_set.assert_called_once_with("success", False)


# ---------------------------------------------------------------------------
# SaveMapNode
# ---------------------------------------------------------------------------

class TestSaveMapNode:
    @pytest.fixture(autouse=True)
    def _setup(self):
        from bteng_nav2.services.map import SaveMapNode
        self.node = make_node(SaveMapNode, map_url="/maps/output_map")

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
        fake_response = MagicMock()
        fake_response.result = True
        with patch.object(self.node, "service_status", return_value=NodeStatus.SUCCESS), \
             patch.object(type(self.node), "service_response",
                          new_callable=lambda: property(lambda self: fake_response)):
            assert self.node.on_running() == NodeStatus.SUCCESS

    def test_make_request_sets_map_url(self):
        req = self.node.make_request()
        assert req.map_url == "/maps/output_map"

    def test_make_request_sets_defaults(self):
        p1, p2, p3 = _start_patched(self.node)
        with p1, p2, p3:
            self.node.on_start()
        req = self.node.make_request()
        assert req.map_topic == "/map"
        assert req.free_thresh == 0.25
        assert req.occupied_thresh == 0.65
        assert req.image_format == "png"
        assert req.map_mode == "trinary"

    def test_on_response_sets_result_output(self):
        fake_response = MagicMock()
        fake_response.result = True
        with patch.object(self.node, "set_output") as mock_set:
            self.node.on_response(fake_response)
        mock_set.assert_called_once_with("result", True)

    def test_on_running_sets_result_output_on_success(self):
        fake_response = MagicMock()
        fake_response.result = True
        with patch.object(self.node, "service_status", return_value=NodeStatus.SUCCESS), \
             patch.object(type(self.node), "service_response",
                          new_callable=lambda: property(lambda self: fake_response)), \
             patch.object(self.node, "set_output") as mock_set:
            status = self.node.on_running()
        assert status == NodeStatus.SUCCESS
        mock_set.assert_called_once_with("result", True)
