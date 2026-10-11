"""Tests for Nav2 recovery, docking, and extended action nodes."""
from __future__ import annotations

from types import SimpleNamespace
from unittest.mock import MagicMock, patch

from bteng import Blackboard, NodeConfig, NodeStatus

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def make_node(cls, **bb_data):
    """Construct a node with a pre-populated blackboard."""
    bb = Blackboard()
    for k, v in bb_data.items():
        bb.set(k, v)
    config = NodeConfig(blackboard=bb, input_ports={k: k for k in bb_data})
    return cls("test", config=config, ros_node=MagicMock())


def _patch_action_client(node):
    """Return a context-manager that patches action-client internals."""
    return patch.object(node, "_init_action_client", return_value=None)


# ===========================================================================
# SpinNode
# ===========================================================================

class TestSpinNode:
    def _make(self, **kw):
        from bteng_nav2.actions.recovery import SpinNode
        return make_node(SpinNode, target_yaw=1.57, **kw)

    def test_on_start_inits_client_and_sends_goal(self):
        node = self._make()
        with patch.object(node, "_init_action_client") as mock_init, \
             patch.object(node, "send_goal") as mock_send:
            node.on_start()
            mock_init.assert_called_once()
            mock_send.assert_called_once()

    def test_on_running_returns_running_while_in_flight(self):
        node = self._make()
        with patch.object(node, "_init_action_client"), \
             patch.object(node, "send_goal"):
            node.on_start()
        node.action_status = MagicMock(return_value=NodeStatus.RUNNING)
        assert node.on_running() == NodeStatus.RUNNING

    def test_on_running_returns_success_when_done(self):
        node = self._make()
        with patch.object(node, "_init_action_client"), \
             patch.object(node, "send_goal"):
            node.on_start()
        node.action_status = MagicMock(return_value=NodeStatus.SUCCESS)
        assert node.on_running() == NodeStatus.SUCCESS

    def test_on_halted_cancels_goal(self):
        node = self._make()
        with patch.object(node, "_init_action_client"), \
             patch.object(node, "send_goal"):
            node.on_start()
        with patch.object(node, "cancel_goal") as mock_cancel:
            node.on_halted()
            mock_cancel.assert_called_once()

    def test_make_goal_sets_fields(self):
        node = self._make(time_allowance=5.0)
        goal = node.make_goal()
        assert goal.target_yaw == 1.57
        assert goal.time_allowance.sec == 5


# ===========================================================================
# BackUpNode
# ===========================================================================

class TestBackUpNode:
    def _make(self, **kw):
        from bteng_nav2.actions.recovery import BackUpNode
        return make_node(BackUpNode, **kw)

    def test_on_start_inits_client_and_sends_goal(self):
        node = self._make()
        with patch.object(node, "_init_action_client") as mock_init, \
             patch.object(node, "send_goal") as mock_send:
            node.on_start()
            mock_init.assert_called_once()
            mock_send.assert_called_once()

    def test_on_running_returns_running_while_in_flight(self):
        node = self._make()
        with patch.object(node, "_init_action_client"), \
             patch.object(node, "send_goal"):
            node.on_start()
        node.action_status = MagicMock(return_value=NodeStatus.RUNNING)
        assert node.on_running() == NodeStatus.RUNNING

    def test_on_running_returns_success_when_done(self):
        node = self._make()
        with patch.object(node, "_init_action_client"), \
             patch.object(node, "send_goal"):
            node.on_start()
        node.action_status = MagicMock(return_value=NodeStatus.SUCCESS)
        assert node.on_running() == NodeStatus.SUCCESS

    def test_on_halted_cancels_goal(self):
        node = self._make()
        with patch.object(node, "_init_action_client"), \
             patch.object(node, "send_goal"):
            node.on_start()
        with patch.object(node, "cancel_goal") as mock_cancel:
            node.on_halted()
            mock_cancel.assert_called_once()

    def test_make_goal_sets_fields(self):
        node = self._make(backup_dist=0.3, backup_speed=0.05, time_allowance=8.0)
        goal = node.make_goal()
        assert goal.target.x == 0.3
        assert goal.speed == 0.05
        assert goal.time_allowance.sec == 8


# ===========================================================================
# DriveOnHeadingNode
# ===========================================================================

class TestDriveOnHeadingNode:
    def _make(self, **kw):
        from bteng_nav2.actions.recovery import DriveOnHeadingNode
        return make_node(DriveOnHeadingNode, dist_to_travel=1.0, speed=0.5, **kw)

    def test_on_start_inits_client_and_sends_goal(self):
        node = self._make()
        with patch.object(node, "_init_action_client") as mock_init, \
             patch.object(node, "send_goal") as mock_send:
            node.on_start()
            mock_init.assert_called_once()
            mock_send.assert_called_once()

    def test_on_running_returns_running_while_in_flight(self):
        node = self._make()
        with patch.object(node, "_init_action_client"), \
             patch.object(node, "send_goal"):
            node.on_start()
        node.action_status = MagicMock(return_value=NodeStatus.RUNNING)
        assert node.on_running() == NodeStatus.RUNNING

    def test_on_running_returns_success_when_done(self):
        node = self._make()
        with patch.object(node, "_init_action_client"), \
             patch.object(node, "send_goal"):
            node.on_start()
        node.action_status = MagicMock(return_value=NodeStatus.SUCCESS)
        assert node.on_running() == NodeStatus.SUCCESS

    def test_on_halted_cancels_goal(self):
        node = self._make()
        with patch.object(node, "_init_action_client"), \
             patch.object(node, "send_goal"):
            node.on_start()
        with patch.object(node, "cancel_goal") as mock_cancel:
            node.on_halted()
            mock_cancel.assert_called_once()

    def test_make_goal_sets_fields(self):
        node = self._make(time_allowance=15.0)
        goal = node.make_goal()
        assert goal.target.x == 1.0
        assert goal.speed == 0.5
        assert goal.time_allowance.sec == 15


# ===========================================================================
# WaitNode
# ===========================================================================

class TestWaitNode:
    def _make(self, **kw):
        from bteng_nav2.actions.recovery import WaitNode
        return make_node(WaitNode, **kw)

    def test_on_start_inits_client_and_sends_goal(self):
        node = self._make()
        with patch.object(node, "_init_action_client") as mock_init, \
             patch.object(node, "send_goal") as mock_send:
            node.on_start()
            mock_init.assert_called_once()
            mock_send.assert_called_once()

    def test_on_running_returns_running_while_in_flight(self):
        node = self._make()
        with patch.object(node, "_init_action_client"), \
             patch.object(node, "send_goal"):
            node.on_start()
        node.action_status = MagicMock(return_value=NodeStatus.RUNNING)
        assert node.on_running() == NodeStatus.RUNNING

    def test_on_running_returns_success_when_done(self):
        node = self._make()
        with patch.object(node, "_init_action_client"), \
             patch.object(node, "send_goal"):
            node.on_start()
        node.action_status = MagicMock(return_value=NodeStatus.SUCCESS)
        assert node.on_running() == NodeStatus.SUCCESS

    def test_on_halted_cancels_goal(self):
        node = self._make()
        with patch.object(node, "_init_action_client"), \
             patch.object(node, "send_goal"):
            node.on_start()
        with patch.object(node, "cancel_goal") as mock_cancel:
            node.on_halted()
            mock_cancel.assert_called_once()

    def test_make_goal_sets_duration(self):
        node = self._make(wait_duration=5)
        goal = node.make_goal()
        assert goal.time.sec == 5

    def test_make_goal_default_duration(self):
        node = self._make()
        goal = node.make_goal()
        assert goal.time.sec == 1


# ===========================================================================
# AssistedTeleopNode
# ===========================================================================

class TestAssistedTeleopNode:
    def _make(self, **kw):
        from bteng_nav2.actions.recovery import AssistedTeleopNode
        return make_node(AssistedTeleopNode, **kw)

    def test_on_start_inits_client_and_sends_goal(self):
        node = self._make()
        with patch.object(node, "_init_action_client") as mock_init, \
             patch.object(node, "send_goal") as mock_send:
            node.on_start()
            mock_init.assert_called_once()
            mock_send.assert_called_once()

    def test_on_running_returns_running_while_in_flight(self):
        node = self._make()
        with patch.object(node, "_init_action_client"), \
             patch.object(node, "send_goal"):
            node.on_start()
        node.action_status = MagicMock(return_value=NodeStatus.RUNNING)
        assert node.on_running() == NodeStatus.RUNNING

    def test_on_running_returns_success_when_done(self):
        node = self._make()
        with patch.object(node, "_init_action_client"), \
             patch.object(node, "send_goal"):
            node.on_start()
        node.action_status = MagicMock(return_value=NodeStatus.SUCCESS)
        assert node.on_running() == NodeStatus.SUCCESS

    def test_on_halted_cancels_goal(self):
        node = self._make()
        with patch.object(node, "_init_action_client"), \
             patch.object(node, "send_goal"):
            node.on_start()
        with patch.object(node, "cancel_goal") as mock_cancel:
            node.on_halted()
            mock_cancel.assert_called_once()

    def test_make_goal_sets_time_allowance(self):
        node = self._make(time_allowance=60.0)
        goal = node.make_goal()
        assert goal.time_allowance.sec == 60

    def test_make_goal_default_time_allowance(self):
        node = self._make()
        goal = node.make_goal()
        assert goal.time_allowance.sec == 30


# ===========================================================================
# DockRobotNode
# ===========================================================================

class TestDockRobotNode:
    def _make(self, **kw):
        from bteng_nav2.actions.docking import DockRobotNode
        return make_node(DockRobotNode, dock_id="dock_01", **kw)

    def test_on_start_inits_client_and_sends_goal(self):
        node = self._make()
        with patch.object(node, "_init_action_client") as mock_init, \
             patch.object(node, "send_goal") as mock_send:
            node.on_start()
            mock_init.assert_called_once()
            mock_send.assert_called_once()

    def test_on_running_returns_running_while_in_flight(self):
        node = self._make()
        with patch.object(node, "_init_action_client"), \
             patch.object(node, "send_goal"):
            node.on_start()
        node.action_status = MagicMock(return_value=NodeStatus.RUNNING)
        assert node.on_running() == NodeStatus.RUNNING

    def test_on_running_returns_success_when_done(self):
        node = self._make()
        with patch.object(node, "_init_action_client"), \
             patch.object(node, "send_goal"):
            node.on_start()
        node._RosActionClientMixin__action_result = SimpleNamespace(
            status=4, result=SimpleNamespace(success=True)
        )
        node.action_status = MagicMock(return_value=NodeStatus.SUCCESS)
        assert node.on_running() == NodeStatus.SUCCESS

    def test_on_halted_cancels_goal(self):
        node = self._make()
        with patch.object(node, "_init_action_client"), \
             patch.object(node, "send_goal"):
            node.on_start()
        with patch.object(node, "cancel_goal") as mock_cancel:
            node.on_halted()
            mock_cancel.assert_called_once()

    def test_on_success_sets_output(self):
        from bteng_nav2.actions.docking import DockRobotNode
        bb = Blackboard()
        bb.set("dock_id", "dock_01")
        config = NodeConfig(
            blackboard=bb,
            input_ports={"dock_id": "dock_id"},
            output_ports={"success": "success"},
        )
        node = DockRobotNode("test", config=config, ros_node=MagicMock())
        node._RosActionClientMixin__action_result = SimpleNamespace(
            status=4, result=SimpleNamespace(success=True)
        )
        node.on_success()
        assert bb.get("success") is True

    def test_make_goal_sets_dock_id_and_type(self):
        node = self._make(dock_type="charging_dock")
        goal = node.make_goal()
        assert goal.dock_id == "dock_01"
        assert goal.dock_type == "charging_dock"


# ===========================================================================
# UndockRobotNode
# ===========================================================================

class TestUndockRobotNode:
    def _make(self, **kw):
        from bteng_nav2.actions.docking import UndockRobotNode
        return make_node(UndockRobotNode, **kw)

    def test_on_start_inits_client_and_sends_goal(self):
        node = self._make()
        with patch.object(node, "_init_action_client") as mock_init, \
             patch.object(node, "send_goal") as mock_send:
            node.on_start()
            mock_init.assert_called_once()
            mock_send.assert_called_once()

    def test_on_running_returns_running_while_in_flight(self):
        node = self._make()
        with patch.object(node, "_init_action_client"), \
             patch.object(node, "send_goal"):
            node.on_start()
        node.action_status = MagicMock(return_value=NodeStatus.RUNNING)
        assert node.on_running() == NodeStatus.RUNNING

    def test_on_running_returns_success_when_done(self):
        node = self._make()
        with patch.object(node, "_init_action_client"), \
             patch.object(node, "send_goal"):
            node.on_start()
        node.action_status = MagicMock(return_value=NodeStatus.SUCCESS)
        assert node.on_running() == NodeStatus.SUCCESS

    def test_on_halted_cancels_goal(self):
        node = self._make()
        with patch.object(node, "_init_action_client"), \
             patch.object(node, "send_goal"):
            node.on_start()
        with patch.object(node, "cancel_goal") as mock_cancel:
            node.on_halted()
            mock_cancel.assert_called_once()

    def test_make_goal_sets_dock_type(self):
        node = self._make(dock_type="charging_dock")
        goal = node.make_goal()
        assert goal.dock_type == "charging_dock"

    def test_make_goal_default_dock_type(self):
        node = self._make()
        goal = node.make_goal()
        assert goal.dock_type == ""
