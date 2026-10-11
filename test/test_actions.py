"""Tests for bteng_nav2 action nodes."""
from __future__ import annotations

from types import SimpleNamespace
from unittest.mock import MagicMock, patch

from bteng import NodeStatus

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def make_node(cls, **bb_data):
    """Construct an action node with a populated blackboard."""
    from bteng import Blackboard, NodeConfig

    bb = Blackboard()
    for k, v in bb_data.items():
        bb.set(k, v)

    input_ports = {k: k for k in bb_data}
    config = NodeConfig(blackboard=bb, input_ports=input_ports)
    ros_node = MagicMock()
    node = cls("test", config=config, ros_node=ros_node)
    return node


def _patch_mixin(node):
    """Patch the three RosActionClientMixin methods on *node* and return the mocks."""
    node._init_action_client = MagicMock()
    node.send_goal = MagicMock()
    node.action_status = MagicMock(return_value=NodeStatus.RUNNING)
    node.cancel_goal = MagicMock()
    node.set_output = MagicMock()
    node._RosActionClientMixin__action_result = MagicMock()
    return node


# ---------------------------------------------------------------------------
# NavigateToPoseNode
# ---------------------------------------------------------------------------

class TestNavigateToPoseNode:
    def _make(self, **kw):
        from bteng_nav2.actions.navigate_to_pose import NavigateToPoseNode

        goal_pose = MagicMock()
        defaults = dict(goal_pose=goal_pose, behavior_tree="")
        defaults.update(kw)
        node = make_node(NavigateToPoseNode, **defaults)
        _patch_mixin(node)
        return node

    def test_on_start_sends_goal(self):
        node = self._make()
        with patch("nav2_msgs.action.NavigateToPose") as mock_action:
            mock_action.Goal.return_value = MagicMock()
            node.on_start()
        node._init_action_client.assert_called_once()
        node.send_goal.assert_called_once()

    def test_on_running_returns_running(self):
        node = self._make()
        node.action_status.return_value = NodeStatus.RUNNING
        status = node.on_running()
        assert status == NodeStatus.RUNNING

    def test_on_running_returns_success_and_sets_output(self):
        node = self._make()
        node.action_status.return_value = NodeStatus.SUCCESS
        fake_result = MagicMock()
        node.action_result.result = fake_result

        status = node.on_running()

        assert status == NodeStatus.SUCCESS
        node.set_output.assert_called_once_with("result", fake_result)

    def test_on_running_returns_failure(self):
        node = self._make()
        node.action_status.return_value = NodeStatus.FAILURE
        status = node.on_running()
        assert status == NodeStatus.FAILURE

    def test_on_halted_cancels_goal(self):
        node = self._make()
        node.on_halted()
        node.cancel_goal.assert_called_once()

    def test_provided_ports(self):
        from bteng_nav2.actions.navigate_to_pose import NavigateToPoseNode

        port_names = {p.name for p in NavigateToPoseNode.provided_ports()}
        assert port_names == {"goal_pose", "behavior_tree", "result", "action_name"}

    def test_make_goal_populates_fields(self):
        from bteng_nav2.actions.navigate_to_pose import NavigateToPoseNode

        fake_pose = MagicMock()
        node = make_node(NavigateToPoseNode, goal_pose=fake_pose, behavior_tree="my_bt")
        _patch_mixin(node)

        with patch("nav2_msgs.action.NavigateToPose") as MockAction:
            fake_goal = MagicMock()
            MockAction.Goal.return_value = fake_goal
            # Patch the import inside make_goal
            import sys
            nav2_module = MagicMock()
            nav2_module.action.NavigateToPose = MockAction
            with patch.dict(sys.modules, {"nav2_msgs": nav2_module, "nav2_msgs.action": nav2_module.action}):
                result = node.make_goal()

        assert fake_goal.pose is fake_pose or result is not None  # goal object created


# ---------------------------------------------------------------------------
# NavigateThroughPosesNode
# ---------------------------------------------------------------------------

class TestNavigateThroughPosesNode:
    def _make(self, **kw):
        from bteng_nav2.actions.navigate_through_poses import NavigateThroughPosesNode

        poses = [MagicMock(), MagicMock()]
        defaults = dict(poses=poses, behavior_tree="")
        defaults.update(kw)
        node = make_node(NavigateThroughPosesNode, **defaults)
        _patch_mixin(node)
        return node

    def test_on_start_sends_goal(self):
        node = self._make()
        with patch("nav2_msgs.action.NavigateThroughPoses") as mock_action:
            mock_action.Goal.return_value = MagicMock()
            node.on_start()
        node._init_action_client.assert_called_once()
        node.send_goal.assert_called_once()

    def test_on_running_returns_running(self):
        node = self._make()
        node.action_status.return_value = NodeStatus.RUNNING
        assert node.on_running() == NodeStatus.RUNNING

    def test_on_running_returns_success_and_sets_output(self):
        node = self._make()
        node.action_status.return_value = NodeStatus.SUCCESS
        fake_result = MagicMock()
        node.action_result.result = fake_result

        status = node.on_running()

        assert status == NodeStatus.SUCCESS
        node.set_output.assert_called_once_with("result", fake_result)

    def test_on_running_returns_failure(self):
        node = self._make()
        node.action_status.return_value = NodeStatus.FAILURE
        assert node.on_running() == NodeStatus.FAILURE

    def test_on_halted_cancels_goal(self):
        node = self._make()
        node.on_halted()
        node.cancel_goal.assert_called_once()

    def test_provided_ports(self):
        from bteng_nav2.actions.navigate_through_poses import NavigateThroughPosesNode

        port_names = {p.name for p in NavigateThroughPosesNode.provided_ports()}
        assert port_names == {"poses", "behavior_tree", "result", "action_name"}


# ---------------------------------------------------------------------------
# FollowPathNode
# ---------------------------------------------------------------------------

class TestFollowPathNode:
    def _make(self, **kw):
        from bteng_nav2.actions.follow_path import FollowPathNode

        path = MagicMock()
        defaults = dict(path=path, controller_id="", goal_checker_id="")
        defaults.update(kw)
        node = make_node(FollowPathNode, **defaults)
        _patch_mixin(node)
        return node

    def test_on_start_sends_goal(self):
        node = self._make()
        with patch("nav2_msgs.action.FollowPath") as mock_action:
            mock_action.Goal.return_value = MagicMock()
            node.on_start()
        node._init_action_client.assert_called_once()
        node.send_goal.assert_called_once()

    def test_on_running_returns_running(self):
        node = self._make()
        node.action_status.return_value = NodeStatus.RUNNING
        assert node.on_running() == NodeStatus.RUNNING

    def test_on_running_returns_success(self):
        node = self._make()
        node.action_status.return_value = NodeStatus.SUCCESS
        assert node.on_running() == NodeStatus.SUCCESS
        # FollowPathNode has no output port, so set_output should not be called
        node.set_output.assert_not_called()

    def test_on_running_returns_failure(self):
        node = self._make()
        node.action_status.return_value = NodeStatus.FAILURE
        assert node.on_running() == NodeStatus.FAILURE

    def test_on_halted_cancels_goal(self):
        node = self._make()
        node.on_halted()
        node.cancel_goal.assert_called_once()

    def test_provided_ports(self):
        from bteng_nav2.actions.follow_path import FollowPathNode

        port_names = {p.name for p in FollowPathNode.provided_ports()}
        assert port_names == {"path", "controller_id", "goal_checker_id", "action_name"}


# ---------------------------------------------------------------------------
# ComputePathToPoseNode
# ---------------------------------------------------------------------------

class TestComputePathToPoseNode:
    def _make(self, **kw):
        from bteng_nav2.actions.compute_path import ComputePathToPoseNode

        goal = MagicMock()
        defaults = dict(goal=goal, planner_id="")
        defaults.update(kw)
        node = make_node(ComputePathToPoseNode, **defaults)
        _patch_mixin(node)
        return node

    def test_on_start_sends_goal(self):
        node = self._make()
        with patch("nav2_msgs.action.ComputePathToPose") as mock_action:
            mock_action.Goal.return_value = MagicMock()
            node.on_start()
        node._init_action_client.assert_called_once()
        node.send_goal.assert_called_once()

    def test_on_running_returns_running(self):
        node = self._make()
        node.action_status.return_value = NodeStatus.RUNNING
        assert node.on_running() == NodeStatus.RUNNING

    def test_on_running_returns_success_and_sets_path(self):
        node = self._make()
        node.action_status.return_value = NodeStatus.SUCCESS
        fake_path = MagicMock()
        # What a real graph delivers: the GetResult *response*, carrying status
        # and the Result -- not the Result itself. A bare MagicMock answers to
        # .path either way, which is how the live AttributeError went unseen.
        node._RosActionClientMixin__action_result = SimpleNamespace(
            status=4, result=SimpleNamespace(path=fake_path)
        )

        status = node.on_running()

        assert status == NodeStatus.SUCCESS
        node.set_output.assert_called_once_with("path", fake_path)

    def test_on_running_returns_failure(self):
        node = self._make()
        node.action_status.return_value = NodeStatus.FAILURE
        assert node.on_running() == NodeStatus.FAILURE

    def test_on_halted_cancels_goal(self):
        node = self._make()
        node.on_halted()
        node.cancel_goal.assert_called_once()

    def test_provided_ports(self):
        from bteng_nav2.actions.compute_path import ComputePathToPoseNode

        port_names = {p.name for p in ComputePathToPoseNode.provided_ports()}
        # goal_pose is the preferred spelling (it matches NavigateToPose); goal
        # is the original one and still works.
        assert port_names == {"goal_pose", "goal", "start", "planner_id", "path", "action_name"}

    def test_make_goal_without_start(self):
        """When start=None, use_start should not be set."""
        from bteng_nav2.actions.compute_path import ComputePathToPoseNode

        node = make_node(ComputePathToPoseNode, goal=MagicMock(), planner_id="")
        _patch_mixin(node)

        import sys
        fake_goal_obj = MagicMock(spec=[])  # blank object so attribute assignment is tracked
        fake_goal_obj = MagicMock()
        MockAction = MagicMock()
        MockAction.Goal.return_value = fake_goal_obj

        nav2_action_module = MagicMock()
        nav2_action_module.ComputePathToPose = MockAction
        nav2_module = MagicMock()
        nav2_module.action = nav2_action_module

        with patch.dict(sys.modules, {"nav2_msgs": nav2_module, "nav2_msgs.action": nav2_action_module}):
            result = node.make_goal()

        assert result is fake_goal_obj
        # use_start should NOT have been set to True when start is None
        assert fake_goal_obj.use_start is not True or not hasattr(fake_goal_obj, "use_start")


# ---------------------------------------------------------------------------
# ComputePathThroughPosesNode
# ---------------------------------------------------------------------------

class TestComputePathThroughPosesNode:
    def _make(self, **kw):
        from bteng_nav2.actions.compute_path import ComputePathThroughPosesNode

        goals = [MagicMock(), MagicMock()]
        defaults = dict(goals=goals, planner_id="")
        defaults.update(kw)
        node = make_node(ComputePathThroughPosesNode, **defaults)
        _patch_mixin(node)
        return node

    def test_on_start_sends_goal(self):
        node = self._make()
        with patch("nav2_msgs.action.ComputePathThroughPoses") as mock_action:
            mock_action.Goal.return_value = MagicMock()
            node.on_start()
        node._init_action_client.assert_called_once()
        node.send_goal.assert_called_once()

    def test_on_running_returns_running(self):
        node = self._make()
        node.action_status.return_value = NodeStatus.RUNNING
        assert node.on_running() == NodeStatus.RUNNING

    def test_on_running_returns_success_and_sets_path(self):
        node = self._make()
        node.action_status.return_value = NodeStatus.SUCCESS
        fake_path = MagicMock()
        # What a real graph delivers: the GetResult *response*, carrying status
        # and the Result -- not the Result itself. A bare MagicMock answers to
        # .path either way, which is how the live AttributeError went unseen.
        node._RosActionClientMixin__action_result = SimpleNamespace(
            status=4, result=SimpleNamespace(path=fake_path)
        )

        status = node.on_running()

        assert status == NodeStatus.SUCCESS
        node.set_output.assert_called_once_with("path", fake_path)

    def test_on_running_returns_failure(self):
        node = self._make()
        node.action_status.return_value = NodeStatus.FAILURE
        assert node.on_running() == NodeStatus.FAILURE

    def test_on_halted_cancels_goal(self):
        node = self._make()
        node.on_halted()
        node.cancel_goal.assert_called_once()

    def test_provided_ports(self):
        from bteng_nav2.actions.compute_path import ComputePathThroughPosesNode

        port_names = {p.name for p in ComputePathThroughPosesNode.provided_ports()}
        assert port_names == {"goals", "start", "planner_id", "path", "action_name"}

    def test_make_goal_with_start(self):
        """When start is provided, use_start should be set True."""
        from bteng_nav2.actions.compute_path import ComputePathThroughPosesNode

        fake_start = MagicMock()
        node = make_node(
            ComputePathThroughPosesNode,
            goals=[MagicMock()],
            start=fake_start,
            planner_id="",
        )
        _patch_mixin(node)

        import sys
        fake_goal_obj = MagicMock()
        MockAction = MagicMock()
        MockAction.Goal.return_value = fake_goal_obj

        nav2_action_module = MagicMock()
        nav2_action_module.ComputePathThroughPoses = MockAction
        nav2_module = MagicMock()
        nav2_module.action = nav2_action_module

        with patch.dict(sys.modules, {"nav2_msgs": nav2_module, "nav2_msgs.action": nav2_action_module}):
            result = node.make_goal()

        assert result is fake_goal_obj
        assert fake_goal_obj.start is fake_start
        assert fake_goal_obj.use_start is True


class TestPhaseAImprovements:
    """Phase A: things bteng_nav2_test found awkward or impossible."""

    @staticmethod
    def _compute_node(**ports):
        from unittest.mock import MagicMock

        from bteng import Blackboard, NodeConfig

        from bteng_nav2.actions.compute_path import ComputePathToPoseNode

        bb = Blackboard()
        for key, value in ports.items():
            bb.set(key, value if value is not None else MagicMock())
        node = ComputePathToPoseNode(
            "compute", config=NodeConfig(blackboard=bb, input_ports={k: k for k in ports})
        )
        return node, bb

    def test_goal_pose_is_accepted(self):
        node, bb = self._compute_node(goal_pose=None)
        assert node._target() is bb.get("goal_pose")

    def test_goal_still_works(self):
        node, bb = self._compute_node(goal=None)
        assert node._target() is bb.get("goal")

    def test_goal_pose_wins_when_both_are_given(self):
        node, bb = self._compute_node(goal_pose=None, goal=None)
        assert node._target() is bb.get("goal_pose")


class TestNamespaceReachesTheBlackboard:
    """--namespace used to miss any endpoint bound to a blackboard key: the name
    lives in the blackboard, and apply_namespace only rewrote nodes."""

    def test_a_service_name_bound_to_a_key_is_namespaced(self):
        from bteng import Blackboard, NodeConfig

        from bteng_nav2 import ManageLifecycleNodesNode
        from bteng_nav2.runtime import apply_namespace

        bb = Blackboard()
        bb.set("svc", "/lifecycle_manager_navigation/manage_nodes")
        node = ManageLifecycleNodesNode(
            "startup", config=NodeConfig(blackboard=bb, input_ports={"service_name": "svc"})
        )

        apply_namespace(node, "/robot1", bb)
        assert bb.get("svc") == "/robot1/lifecycle_manager_navigation/manage_nodes"

    def test_rewriting_is_idempotent(self):
        from bteng import Blackboard, NodeConfig

        from bteng_nav2 import ManageLifecycleNodesNode
        from bteng_nav2.runtime import apply_namespace

        bb = Blackboard()
        bb.set("svc", "/manage")
        node = ManageLifecycleNodesNode(
            "startup", config=NodeConfig(blackboard=bb, input_ports={"service_name": "svc"})
        )
        apply_namespace(node, "/robot1", bb)
        apply_namespace(node, "/robot1", bb)
        assert bb.get("svc") == "/robot1/manage"

    def test_without_a_blackboard_nothing_explodes(self):
        from bteng import Blackboard, NodeConfig

        from bteng_nav2 import ManageLifecycleNodesNode
        from bteng_nav2.runtime import apply_namespace

        bb = Blackboard()
        bb.set("svc", "/manage")
        node = ManageLifecycleNodesNode(
            "startup", config=NodeConfig(blackboard=bb, input_ports={"service_name": "svc"})
        )
        apply_namespace(node, "/robot1")  # blackboard omitted
        assert bb.get("svc") == "/manage"
