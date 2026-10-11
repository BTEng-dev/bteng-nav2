"""Integration tests for bteng_nav2 action nodes using FakeRosNode.

These tests exercise the real RosActionClientMixin code path — no mocking of
bteng_ros2 internals. rclpy.action.ActionClient is replaced with
FakeActionClient / SlowActionClient from bteng_ros2.testing so goals resolve
without a running ROS 2 middleware.
"""
from __future__ import annotations

import sys

from bteng import Blackboard, NodeConfig, NodeStatus
from bteng_ros2.testing import FakeActionClient, FakeRosNode, SlowActionClient

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_config(**bb_data):
    bb = Blackboard()
    for k, v in bb_data.items():
        bb.set(k, v)
    return NodeConfig(blackboard=bb, input_ports={k: k for k in bb_data})


def _install_action_client(cls):
    """Patch rclpy.action.ActionClient with *cls* for the duration of a test."""
    import rclpy.action as _action_mod
    old = _action_mod.ActionClient
    _action_mod.ActionClient = cls
    sys.modules["rclpy.action"].ActionClient = cls
    return old


def _restore_action_client(old):
    import rclpy.action as _action_mod
    _action_mod.ActionClient = old
    sys.modules["rclpy.action"].ActionClient = old


# ---------------------------------------------------------------------------
# NavigateToPoseNode
# ---------------------------------------------------------------------------

class TestNavigateToPoseNodeIntegration:
    """End-to-end tests for NavigateToPoseNode with FakeRosNode."""

    def _make_node(self, succeed=True):
        from geometry_msgs.msg import PoseStamped

        from bteng_nav2.actions.navigate_to_pose import NavigateToPoseNode

        goal_pose = PoseStamped()
        config = _make_config(goal_pose=goal_pose, behavior_tree="")
        fake = FakeRosNode()
        node = NavigateToPoseNode("nav_test", config=config, ros_node=fake)
        return node, fake

    def test_navigate_to_pose_success(self):
        """NavigateToPoseNode completes with SUCCESS and writes the result output."""
        node, _ = self._make_node(succeed=True)

        old = _install_action_client(
            lambda n, t, name: FakeActionClient(n, t, name, succeed=True)
        )
        try:
            node.on_start()
            run_status = node.on_running()
            assert run_status == NodeStatus.SUCCESS
        finally:
            _restore_action_client(old)

    def test_navigate_to_pose_failure(self):
        """NavigateToPoseNode returns FAILURE when the action is aborted."""
        node, _ = self._make_node(succeed=False)

        old = _install_action_client(
            lambda n, t, name: FakeActionClient(n, t, name, succeed=False)
        )
        try:
            node.on_start()
            run_status = node.on_running()
            assert run_status == NodeStatus.FAILURE
        finally:
            _restore_action_client(old)

    def test_navigate_to_pose_output_port_written(self):
        """on_success() writes the nav result to the 'result' output port."""
        from geometry_msgs.msg import PoseStamped

        from bteng_nav2.actions.navigate_to_pose import NavigateToPoseNode

        bb = Blackboard()
        bb.set("goal_pose", PoseStamped())
        bb.set("behavior_tree", "")
        config = NodeConfig(
            blackboard=bb,
            input_ports={"goal_pose": "goal_pose", "behavior_tree": "behavior_tree"},
            output_ports={"result": "nav_result"},
        )
        fake = FakeRosNode()
        node = NavigateToPoseNode("nav_test", config=config, ros_node=fake)

        old = _install_action_client(
            lambda n, t, name: FakeActionClient(n, t, name, succeed=True)
        )
        try:
            node.on_start()
            status = node.on_running()
            assert status == NodeStatus.SUCCESS
            # The output port "result" should have been written to the blackboard
            result_val = bb.get("nav_result")
            assert result_val is not None
        finally:
            _restore_action_client(old)

    def test_navigate_to_pose_halted_cancels_goal(self):
        """on_halted() cancels the in-flight goal via cancel_goal_async."""
        node, _ = self._make_node()

        old = _install_action_client(
            lambda n, t, name: SlowActionClient(n, t, name)
        )
        try:
            node.on_start()
            slow_client = node._RosActionClientMixin__action_client
            # goal acceptance is deferred — state is SENDING → RUNNING
            assert node.on_running() == NodeStatus.RUNNING
            # Resolve acceptance so goal handle is set and state transitions to RUNNING
            slow_client._pending.resolve()
            assert node.on_running() == NodeStatus.RUNNING
            node.on_halted()
            # Now goal handle exists and state was RUNNING → cancel_goal_async called
            pending_handle = slow_client._pending._result
            assert pending_handle.cancelled is True
        finally:
            _restore_action_client(old)


# ---------------------------------------------------------------------------
# ComputePathToPoseNode
# ---------------------------------------------------------------------------

class TestComputePathToPoseNodeIntegration:
    """End-to-end tests for ComputePathToPoseNode with FakeRosNode."""

    def _make_node(self, succeed=True):
        from geometry_msgs.msg import PoseStamped

        from bteng_nav2.actions.compute_path import ComputePathToPoseNode

        goal_pose = PoseStamped()
        bb = Blackboard()
        bb.set("goal", goal_pose)
        bb.set("planner_id", "")
        config = NodeConfig(
            blackboard=bb,
            input_ports={"goal": "goal", "planner_id": "planner_id"},
            output_ports={"path": "computed_path"},
        )
        fake = FakeRosNode()
        node = ComputePathToPoseNode("cpt_test", config=config, ros_node=fake)
        return node, bb, fake

    def test_compute_path_to_pose_success(self):
        """ComputePathToPoseNode completes with SUCCESS."""
        node, bb, _ = self._make_node(succeed=True)

        old = _install_action_client(
            lambda n, t, name: FakeActionClient(n, t, name, succeed=True)
        )
        try:
            node.on_start()
            status = node.on_running()
            assert status == NodeStatus.SUCCESS
        finally:
            _restore_action_client(old)

    def test_compute_path_to_pose_writes_path(self):
        """ComputePathToPoseNode writes the path result to the blackboard."""
        node, bb, _ = self._make_node(succeed=True)

        old = _install_action_client(
            lambda n, t, name: FakeActionClient(n, t, name, succeed=True)
        )
        try:
            node.on_start()
            status = node.on_running()
            assert status == NodeStatus.SUCCESS
            # The output port "path" should have been written
            path_val = bb.get("computed_path")
            assert path_val is not None
        finally:
            _restore_action_client(old)

    def test_compute_path_to_pose_failure(self):
        """ComputePathToPoseNode returns FAILURE when action is aborted."""
        node, bb, _ = self._make_node(succeed=False)

        old = _install_action_client(
            lambda n, t, name: FakeActionClient(n, t, name, succeed=False)
        )
        try:
            node.on_start()
            status = node.on_running()
            assert status == NodeStatus.FAILURE
        finally:
            _restore_action_client(old)
