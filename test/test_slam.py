"""Tests for SLAM Toolbox service nodes."""
from __future__ import annotations

import sys
import types as _types

# ---------------------------------------------------------------------------
# slam_toolbox stubs — must be registered before importing bteng_nav2.services.slam
# ---------------------------------------------------------------------------

_slam_pkg = _types.ModuleType("slam_toolbox")
_slam_srv = _types.ModuleType("slam_toolbox.srv")


class _SlamSaveMap:
    class Request:
        def __init__(self):
            self.name = ""

    class Response:
        pass


class _SerializePoseGraph:
    class Request:
        def __init__(self):
            self.filename = ""

    class Response:
        pass


_slam_srv.SaveMap = _SlamSaveMap
_slam_srv.SerializePoseGraph = _SerializePoseGraph
_slam_pkg.srv = _slam_srv
sys.modules.setdefault("slam_toolbox", _slam_pkg)
sys.modules["slam_toolbox.srv"] = _slam_srv

# ---------------------------------------------------------------------------
# Standard imports
# ---------------------------------------------------------------------------

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
    """Return patches so on_start() runs without a live ROS environment."""
    return (
        patch.object(node, "_init_service_client"),
        patch.object(node, "call_service"),
        patch.object(node, "service_is_ready", return_value=True),
    )


# ---------------------------------------------------------------------------
# SaveSlamMapNode
# ---------------------------------------------------------------------------

class TestSaveSlamMapNode:
    @pytest.fixture(autouse=True)
    def _setup(self):
        from bteng_nav2.services.slam import SaveSlamMapNode
        self.node = make_node(SaveSlamMapNode, filename="my_map")

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

    def test_make_request_sets_filename(self):
        req_mock = MagicMock()
        req_cls_mock = MagicMock(return_value=req_mock)
        with patch("slam_toolbox.srv.SaveMap") as srv_mock:
            srv_mock.Request = req_cls_mock
            p1, p2, p3 = _start_patched(self.node)
            with p1, p2, p3:
                self.node.on_start()
        # slam_toolbox declares `name` as std_msgs/String -- a message, not a
        # str. Asserting the bare string here is what let the live abort
        # through: rclpy aborts the process on the wrong type, it does not raise.
        assert req_mock.name.data == "my_map"


# ---------------------------------------------------------------------------
# SerializePoseGraphNode
# ---------------------------------------------------------------------------

class TestSerializePoseGraphNode:
    @pytest.fixture(autouse=True)
    def _setup(self):
        from bteng_nav2.services.slam import SerializePoseGraphNode
        self.node = make_node(SerializePoseGraphNode, filename="my_map")

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

    def test_make_request_sets_filename(self):
        req_mock = MagicMock()
        req_cls_mock = MagicMock(return_value=req_mock)
        with patch("slam_toolbox.srv.SerializePoseGraph") as srv_mock:
            srv_mock.Request = req_cls_mock
            p1, p2, p3 = _start_patched(self.node)
            with p1, p2, p3:
                self.node.on_start()
        assert req_mock.filename == "my_map"
