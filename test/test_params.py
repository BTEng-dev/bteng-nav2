"""Tests for SetCostmapParameterNode and GetCostmapParameterNode."""
from __future__ import annotations

import sys
import types as _types

# ---------------------------------------------------------------------------
# Stubs for rcl_interfaces.srv (conftest only stubs rcl_interfaces.msg)
# ---------------------------------------------------------------------------

_rcl_srv_mod = _types.ModuleType("rcl_interfaces.srv")


class _SetParameters:
    class Request:
        def __init__(self):
            self.parameters = []

    class Response:
        def __init__(self):
            from unittest.mock import MagicMock
            self.results = [MagicMock(successful=True, reason="")]


class _GetParameters:
    class Request:
        def __init__(self):
            self.names = []

    class Response:
        def __init__(self):
            pv = _types.SimpleNamespace(
                type=3,
                double_value=0.0,
                string_value="",
                bool_value=False,
                integer_value=0,
            )
            self.values = [pv]


_rcl_srv_mod.SetParameters = _SetParameters
_rcl_srv_mod.GetParameters = _GetParameters
sys.modules["rcl_interfaces.srv"] = _rcl_srv_mod

# ---------------------------------------------------------------------------
# Add Parameter and ParameterValue to rcl_interfaces.msg
# ---------------------------------------------------------------------------

_rcl_msg_mod = sys.modules.get("rcl_interfaces.msg", _types.ModuleType("rcl_interfaces.msg"))


class _Parameter:
    def __init__(self):
        self.name = ""
        self.value = None


class _ParameterValue:
    def __init__(self):
        self.type = 0
        self.double_value = 0.0
        self.string_value = ""
        self.bool_value = False
        self.integer_value = 0


_rcl_msg_mod.Parameter = _Parameter
_rcl_msg_mod.ParameterValue = _ParameterValue
sys.modules["rcl_interfaces.msg"] = _rcl_msg_mod

# ---------------------------------------------------------------------------
# Standard imports
# ---------------------------------------------------------------------------

from unittest.mock import MagicMock, patch

import pytest
from bteng import Blackboard, NodeConfig, NodeStatus

# ---------------------------------------------------------------------------
# Helpers (mirrors test_services.py pattern)
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
    return (
        patch.object(node, "_init_service_client"),
        patch.object(node, "call_service"),
        patch.object(node, "service_is_ready", return_value=True),
    )


# ---------------------------------------------------------------------------
# SetCostmapParameterNode tests
# ---------------------------------------------------------------------------

class TestSetCostmapParameterNode:
    @pytest.fixture(autouse=True)
    def _setup(self):
        from bteng_nav2.services.params import SetCostmapParameterNode
        self.node = make_node(
            SetCostmapParameterNode,
            ros_node_name="global_costmap/global_costmap",
            param_name="inflation_layer.inflation_radius",
            param_value=0.5,
        )

    def test_on_start_sets_dynamic_service_name(self):
        p1, p2, p3 = _start_patched(self.node)
        with p1, p2, p3:
            self.node.on_start()
        assert self.node.service_name == "/global_costmap/global_costmap/set_parameters"

    def test_on_start_initializes_and_calls_service(self):
        p1, p2, p3 = _start_patched(self.node)
        with p1 as mock_init, p2 as mock_call, p3:
            result = self.node.on_start()
        mock_init.assert_called_once()
        mock_call.assert_called_once()
        assert result == NodeStatus.RUNNING

    def test_on_running_returns_running(self):
        with patch.object(self.node, "service_status", return_value=NodeStatus.RUNNING):
            assert self.node.on_running() == NodeStatus.RUNNING

    def test_on_running_returns_success(self):
        with patch.object(self.node, "service_status", return_value=NodeStatus.SUCCESS), \
             patch.object(type(self.node), "service_response",
                          new_callable=lambda: property(lambda self: MagicMock())):
            assert self.node.on_running() == NodeStatus.SUCCESS

    def test_make_request_sets_param_name_and_value(self):
        req = self.node.make_request()
        assert req.parameters[0].name == "inflation_layer.inflation_radius"
        assert req.parameters[0].value.double_value == pytest.approx(0.5)


# ---------------------------------------------------------------------------
# GetCostmapParameterNode tests
# ---------------------------------------------------------------------------

class TestGetCostmapParameterNode:
    @pytest.fixture(autouse=True)
    def _setup(self):
        from bteng_nav2.services.params import GetCostmapParameterNode
        self.node = make_node(
            GetCostmapParameterNode,
            ros_node_name="local_costmap/local_costmap",
            param_name="my_param",
        )

    def test_on_start_sets_dynamic_service_name(self):
        p1, p2, p3 = _start_patched(self.node)
        with p1, p2, p3:
            self.node.on_start()
        assert self.node.service_name == "/local_costmap/local_costmap/get_parameters"

    def test_on_start_initializes_and_calls_service(self):
        p1, p2, p3 = _start_patched(self.node)
        with p1 as mock_init, p2 as mock_call, p3:
            result = self.node.on_start()
        mock_init.assert_called_once()
        mock_call.assert_called_once()
        assert result == NodeStatus.RUNNING

    def test_on_running_returns_running(self):
        with patch.object(self.node, "service_status", return_value=NodeStatus.RUNNING):
            assert self.node.on_running() == NodeStatus.RUNNING

    def test_on_running_sets_param_value_output(self):
        fake_pv = _types.SimpleNamespace(double_value=0.3)
        fake_response = _types.SimpleNamespace(values=[fake_pv])
        with patch.object(self.node, "service_status", return_value=NodeStatus.SUCCESS), \
             patch.object(type(self.node), "service_response",
                          new_callable=lambda: property(lambda self: fake_response)), \
             patch.object(self.node, "set_output") as mock_set:
            self.node.on_running()
        mock_set.assert_called_once_with("param_value", 0.3)

    def test_make_request_sets_param_name(self):
        req = self.node.make_request()
        assert req.names == ["my_param"]
