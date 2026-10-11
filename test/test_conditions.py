"""Tests for bteng_nav2 condition nodes."""
from __future__ import annotations

import time
from unittest.mock import MagicMock, patch

from bteng import Blackboard, NodeConfig, NodeStatus

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def make_node(cls, **bb_data):
    """Construct a node wired to a fresh Blackboard."""
    from bteng_ros2 import RosNodeMixin
    bb = Blackboard()
    for k, v in bb_data.items():
        bb.set(k, v)
    config = NodeConfig(blackboard=bb, input_ports={k: k for k in bb_data})
    if issubclass(cls, RosNodeMixin):
        return cls("test", config=config, ros_node=MagicMock())
    return cls("test", config=config)


def _make_pose_msg(x=0.0, y=0.0):
    """Build a minimal PoseWithCovarianceStamped-like mock."""
    msg = MagicMock()
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    return msg


def _make_goal_pose(x=0.0, y=0.0):
    """Build a minimal PoseStamped-like mock."""
    goal = MagicMock()
    goal.pose.position.x = x
    goal.pose.position.y = y
    return goal


# ---------------------------------------------------------------------------
# IsRobotNearGoalCondition
# ---------------------------------------------------------------------------

class TestIsRobotNearGoalCondition:
    def _make(self, goal_pose=None, tolerance=0.5):
        from bteng_nav2.conditions.pose import IsRobotNearGoalCondition

        if goal_pose is None:
            goal_pose = _make_goal_pose(1.0, 1.0)
        return make_node(
            IsRobotNearGoalCondition,
            goal_pose=goal_pose,
            tolerance=tolerance,
        )

    def test_evaluate_near_returns_true(self):
        goal = _make_goal_pose(1.0, 1.0)
        node = self._make(goal_pose=goal, tolerance=0.5)
        # robot at (1.0, 1.0) — distance 0.0 < 0.5
        msg = _make_pose_msg(1.0, 1.0)
        assert node.evaluate(msg) is True

    def test_evaluate_far_returns_false(self):
        goal = _make_goal_pose(0.0, 0.0)
        node = self._make(goal_pose=goal, tolerance=0.5)
        # robot at (10.0, 0.0) — distance 10.0 > 0.5
        msg = _make_pose_msg(10.0, 0.0)
        assert node.evaluate(msg) is False

    def test_evaluate_exactly_at_tolerance_boundary(self):
        goal = _make_goal_pose(0.0, 0.0)
        node = self._make(goal_pose=goal, tolerance=1.0)
        # robot at (0.99, 0.0) — distance 0.99 < 1.0  → True
        msg = _make_pose_msg(0.99, 0.0)
        assert node.evaluate(msg) is True

    def test_tick_sets_topic_type_lazily(self):
        goal = _make_goal_pose(0.0, 0.0)
        node = self._make(goal_pose=goal)
        assert node.topic_type is None
        # Patch super().tick() to avoid actual ROS subscription logic
        with patch("bteng_ros2.RosConditionNode.tick", return_value=NodeStatus.SUCCESS):
            node.tick()
        from geometry_msgs.msg import PoseWithCovarianceStamped
        assert node.topic_type is PoseWithCovarianceStamped


# ---------------------------------------------------------------------------
# IsLocalizationActiveCondition
# ---------------------------------------------------------------------------

class TestIsLocalizationActiveCondition:
    def _make(self, timeout=2.0):
        from bteng_nav2.conditions.pose import IsLocalizationActiveCondition

        return make_node(IsLocalizationActiveCondition, timeout=timeout)

    def test_no_msg_received_returns_false(self):
        node = self._make(timeout=2.0)
        # _last_msg_time is -inf, so time.monotonic() - (-inf) is always > timeout
        msg = MagicMock()
        assert node.evaluate(msg) is False

    def test_recent_msg_returns_true(self):
        node = self._make(timeout=2.0)
        node._last_msg_time = time.monotonic()
        msg = MagicMock()
        assert node.evaluate(msg) is True

    def test_stale_msg_returns_false(self):
        node = self._make(timeout=1.0)
        # Simulate a message received 5 seconds ago
        node._last_msg_time = time.monotonic() - 5.0
        msg = MagicMock()
        assert node.evaluate(msg) is False

    def test_on_msg_updates_last_msg_time(self):
        node = self._make(timeout=2.0)
        assert node._last_msg_time == float("-inf")
        before = time.monotonic()
        node._on_msg(MagicMock())
        after = time.monotonic()
        assert before <= node._last_msg_time <= after

    def test_tick_sets_topic_type_lazily(self):
        node = self._make()
        assert node.topic_type is None
        with patch("bteng_ros2.RosConditionNode.tick", return_value=NodeStatus.SUCCESS):
            node.tick()
        from geometry_msgs.msg import PoseWithCovarianceStamped
        assert node.topic_type is PoseWithCovarianceStamped


# ---------------------------------------------------------------------------
# IsObstacleNearCondition
# ---------------------------------------------------------------------------

class TestIsObstacleNearCondition:
    def _make(self, min_distance=0.5):
        from bteng_nav2.conditions.obstacle import IsObstacleNearCondition

        return make_node(IsObstacleNearCondition, min_distance=min_distance)

    def _make_scan(self, ranges):
        msg = MagicMock()
        msg.ranges = ranges
        return msg

    def test_obstacle_near_returns_true(self):
        node = self._make(min_distance=1.0)
        msg = self._make_scan([5.0, 0.3, 4.0])
        assert node.evaluate(msg) is True

    def test_obstacle_far_returns_false(self):
        node = self._make(min_distance=0.5)
        msg = self._make_scan([2.0, 3.0, 5.0])
        assert node.evaluate(msg) is False

    def test_empty_ranges_returns_false(self):
        node = self._make(min_distance=0.5)
        msg = self._make_scan([])
        assert node.evaluate(msg) is False

    def test_all_zero_ranges_returns_false(self):
        # Ranges of 0 are filtered out (invalid readings)
        node = self._make(min_distance=0.5)
        msg = self._make_scan([0.0, 0.0, 0.0])
        assert node.evaluate(msg) is False

    def test_tick_sets_topic_type_lazily(self):
        node = self._make()
        assert node.topic_type is None
        with patch("bteng_ros2.RosConditionNode.tick", return_value=NodeStatus.SUCCESS):
            node.tick()
        from sensor_msgs.msg import LaserScan
        assert node.topic_type is LaserScan


# ---------------------------------------------------------------------------
# IsStuckCondition
# ---------------------------------------------------------------------------

class TestIsStuckCondition:
    def _make(self, stuck_timeout=3.0):
        from bteng_nav2.conditions.obstacle import IsStuckCondition

        return make_node(IsStuckCondition, stuck_timeout=stuck_timeout)

    def _make_odom(self, vx=0.0, wz=0.0):
        msg = MagicMock()
        msg.twist.twist.linear.x = vx
        msg.twist.twist.angular.z = wz
        return msg

    def test_moving_returns_false_and_clears_stuck_since(self):
        node = self._make(stuck_timeout=3.0)
        msg = self._make_odom(vx=0.5, wz=0.0)
        assert node.evaluate(msg) is False
        assert node._stuck_since is None

    def test_stationary_below_timeout_returns_false(self):
        node = self._make(stuck_timeout=10.0)
        msg = self._make_odom(vx=0.0, wz=0.0)
        # First call: records stuck_since, but timeout not elapsed
        assert node.evaluate(msg) is False
        assert node._stuck_since is not None

    def test_stationary_above_timeout_returns_true(self):
        node = self._make(stuck_timeout=3.0)
        node._stuck_since = time.monotonic() - 10.0  # 10 s ago
        msg = self._make_odom(vx=0.0, wz=0.0)
        assert node.evaluate(msg) is True

    def test_moving_after_stuck_clears_timer(self):
        node = self._make(stuck_timeout=3.0)
        node._stuck_since = time.monotonic() - 10.0
        # Now robot moves
        msg = self._make_odom(vx=0.3, wz=0.0)
        assert node.evaluate(msg) is False
        assert node._stuck_since is None

    def test_tick_sets_topic_type_lazily(self):
        node = self._make()
        assert node.topic_type is None
        with patch("bteng_ros2.RosConditionNode.tick", return_value=NodeStatus.SUCCESS):
            node.tick()
        from nav_msgs.msg import Odometry
        assert node.topic_type is Odometry


# ---------------------------------------------------------------------------
# IsGoalUpdatedCondition
# ---------------------------------------------------------------------------

class TestIsGoalUpdatedCondition:
    def _make(self, goal_pose=None):
        from bteng_nav2.conditions.nav_status import IsGoalUpdatedCondition

        if goal_pose is None:
            goal_pose = MagicMock()
        return make_node(IsGoalUpdatedCondition, goal_pose=goal_pose)

    def test_first_call_returns_success(self):
        goal = MagicMock()
        goal.__str__ = lambda s: "goal_a"
        node = self._make(goal_pose=goal)
        assert node.tick() == NodeStatus.SUCCESS

    def test_same_goal_second_call_returns_failure(self):
        goal = MagicMock()
        goal.__str__ = lambda s: "goal_a"
        node = self._make(goal_pose=goal)
        node.tick()  # first call: SUCCESS and stores hash
        assert node.tick() == NodeStatus.FAILURE

    def test_changed_goal_returns_success(self):
        goal_a = MagicMock()
        goal_a.__str__ = lambda s: "goal_a"
        node = self._make(goal_pose=goal_a)
        node.tick()  # SUCCESS, stores hash of goal_a

        # Now change the blackboard value
        node.blackboard.set("goal_pose", MagicMock())
        goal_b = MagicMock()
        goal_b.__str__ = lambda s: "goal_b"
        node.blackboard.set("goal_pose", goal_b)

        assert node.tick() == NodeStatus.SUCCESS


# ---------------------------------------------------------------------------
# IsNavigationActiveCondition
# ---------------------------------------------------------------------------

class TestIsNavigationActiveCondition:
    def _make(self):
        from bteng_nav2.conditions.nav_status import IsNavigationActiveCondition

        return make_node(IsNavigationActiveCondition)

    def test_server_ready_returns_success(self):
        node = self._make()
        mock_client = MagicMock()
        mock_client.server_is_ready.return_value = True
        node._client = mock_client
        assert node.tick() == NodeStatus.SUCCESS

    def test_server_not_ready_returns_failure(self):
        node = self._make()
        mock_client = MagicMock()
        mock_client.server_is_ready.return_value = False
        node._client = mock_client
        assert node.tick() == NodeStatus.FAILURE

    def test_creates_client_on_first_tick(self):
        node = self._make()
        assert node._client is None

        mock_action_client_cls = MagicMock()
        mock_client_instance = MagicMock()
        mock_client_instance.server_is_ready.return_value = True
        mock_action_client_cls.return_value = mock_client_instance

        with patch("rclpy.action.ActionClient", mock_action_client_cls):
            with patch("nav2_msgs.action.NavigateToPose", MagicMock()):
                result = node.tick()

        assert node._client is mock_client_instance
        assert result == NodeStatus.SUCCESS

    def test_reuses_client_on_subsequent_ticks(self):
        node = self._make()
        mock_client = MagicMock()
        mock_client.server_is_ready.return_value = True
        node._client = mock_client

        node.tick()
        node.tick()

        # ActionClient constructor should NOT have been called again
        assert mock_client.server_is_ready.call_count == 2


class _FakeRosNode:
    def create_subscription(self, *a, **kw):
        return object()

    def get_logger(self):
        import logging

        return logging.getLogger("fake")


class TestEmergencyStopTopicPort:
    """The topic_name port used to bind cleanly and do nothing: RosConditionNode
    subscribes to the class attribute, so an XML override was silently ignored."""

    @staticmethod
    def _node(**ports):
        from bteng import Blackboard, NodeConfig

        from bteng_nav2 import IsEmergencyStopCondition

        bb = Blackboard()
        cfg = NodeConfig(blackboard=bb, params=dict(ports))
        node = IsEmergencyStopCondition("estop", config=cfg)
        node.set_ros_node(_FakeRosNode())
        return node

    def test_port_overrides_the_class_attribute(self):
        node = self._node(topic_name="/robot1/estop")
        node.tick()
        assert node.topic_name == "/robot1/estop"

    def test_default_is_used_when_the_port_is_absent(self):
        node = self._node()
        node.tick()
        assert node.topic_name == "/emergency_stop"
