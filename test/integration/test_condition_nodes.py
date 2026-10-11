"""Integration tests for bteng_nav2 condition nodes using FakeRosNode.

Condition nodes subscribe to a ROS topic and evaluate each received message.
FakeRosNode.subscriptions[topic].inject(msg) fires the real subscription
callback, exercising the full RosConditionNode / RosTopicMixin pipeline.
"""
from __future__ import annotations

from unittest.mock import MagicMock

from bteng import Blackboard, NodeConfig, NodeStatus
from bteng_ros2.testing import FakeRosNode

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_config(**bb_data):
    bb = Blackboard()
    for k, v in bb_data.items():
        bb.set(k, v)
    return bb, NodeConfig(blackboard=bb, input_ports={k: k for k in bb_data})


def _make_scan_msg(ranges):
    msg = MagicMock()
    msg.ranges = ranges
    return msg


def _make_pose_msg(x=0.0, y=0.0):
    msg = MagicMock()
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    return msg


def _make_goal_pose(x=0.0, y=0.0):
    goal = MagicMock()
    goal.pose.position.x = x
    goal.pose.position.y = y
    return goal


# ---------------------------------------------------------------------------
# IsObstacleNearCondition
# ---------------------------------------------------------------------------

class TestIsObstacleNearIntegration:
    """FakeRosNode subscription injection exercises evaluate() end-to-end."""

    def _make_node(self, min_distance=0.5):
        from bteng_nav2.conditions.obstacle import IsObstacleNearCondition

        fake = FakeRosNode()
        bb, config = _make_config(min_distance=min_distance)
        node = IsObstacleNearCondition("obs_test", config=config, ros_node=fake)
        return node, fake

    def test_obstacle_near_no_message(self):
        """Before any message is received the node returns FAILURE."""
        node, _ = self._make_node(min_distance=0.5)
        # First tick initialises the subscription but no message yet
        status = node.tick()
        assert status == NodeStatus.FAILURE

    def test_obstacle_near_detected(self):
        """After injecting a close scan, the node returns SUCCESS."""
        node, fake = self._make_node(min_distance=0.5)

        # Warm up: first tick creates the subscription
        node.tick()

        # Inject a scan with one range inside the threshold
        msg = _make_scan_msg([0.3, 0.4, 0.2])
        fake.subscriptions["/scan"].inject(msg)

        assert node.tick() == NodeStatus.SUCCESS

    def test_obstacle_not_near(self):
        """After injecting a far scan, the node returns FAILURE."""
        node, fake = self._make_node(min_distance=0.5)

        node.tick()

        msg = _make_scan_msg([2.0, 3.0, 5.0])
        fake.subscriptions["/scan"].inject(msg)

        assert node.tick() == NodeStatus.FAILURE

    def test_obstacle_near_threshold_boundary(self):
        """A range exactly at the threshold is NOT near (strict less-than)."""
        node, fake = self._make_node(min_distance=1.0)

        node.tick()

        # min(ranges) == 1.0 — not < 1.0 → FAILURE
        msg = _make_scan_msg([1.0, 2.0, 3.0])
        fake.subscriptions["/scan"].inject(msg)

        assert node.tick() == NodeStatus.FAILURE

    def test_obstacle_all_zero_ranges(self):
        """Zero ranges are filtered as invalid — node returns FAILURE."""
        node, fake = self._make_node(min_distance=0.5)

        node.tick()

        msg = _make_scan_msg([0.0, 0.0, 0.0])
        fake.subscriptions["/scan"].inject(msg)

        assert node.tick() == NodeStatus.FAILURE


# ---------------------------------------------------------------------------
# IsLocalizationActiveCondition
# ---------------------------------------------------------------------------

class TestIsLocalizationActiveIntegration:
    """Localization condition — becomes active after a pose message is received."""

    def _make_node(self, timeout=2.0):
        from bteng_nav2.conditions.pose import IsLocalizationActiveCondition

        fake = FakeRosNode()
        bb, config = _make_config(timeout=timeout)
        node = IsLocalizationActiveCondition("loc_test", config=config, ros_node=fake)
        return node, fake

    def test_localization_inactive_before_message(self):
        """No message received → FAILURE (last_msg_time is -inf)."""
        node, _ = self._make_node(timeout=2.0)
        node.tick()
        assert node.tick() == NodeStatus.FAILURE

    def test_localization_active_after_message(self):
        """After injecting a pose message, the condition returns SUCCESS."""
        node, fake = self._make_node(timeout=2.0)

        # Warm up to create subscription
        node.tick()

        pose_msg = _make_pose_msg(0.0, 0.0)
        fake.subscriptions["/amcl_pose"].inject(pose_msg)

        assert node.tick() == NodeStatus.SUCCESS

    def test_localization_subscription_registered(self):
        """First tick registers the /amcl_pose subscription on FakeRosNode."""
        node, fake = self._make_node()
        node.tick()
        assert "/amcl_pose" in fake.subscriptions


# ---------------------------------------------------------------------------
# IsRobotNearGoalCondition
# ---------------------------------------------------------------------------

class TestIsRobotNearGoalIntegration:
    """Robot-near-goal condition — uses injected amcl_pose."""

    def _make_node(self, goal_x=2.0, goal_y=2.0, tolerance=0.5):
        from bteng_nav2.conditions.pose import IsRobotNearGoalCondition

        goal = _make_goal_pose(goal_x, goal_y)
        fake = FakeRosNode()
        bb, config = _make_config(goal_pose=goal, tolerance=tolerance)
        node = IsRobotNearGoalCondition("near_test", config=config, ros_node=fake)
        return node, fake

    def test_robot_near_goal(self):
        """Robot within tolerance of goal → SUCCESS."""
        node, fake = self._make_node(goal_x=2.0, goal_y=2.0, tolerance=0.5)

        node.tick()  # create subscription

        # Robot at (1.9, 1.95) — distance ≈ 0.11 < 0.5
        msg = _make_pose_msg(x=1.9, y=1.95)
        fake.subscriptions["/amcl_pose"].inject(msg)

        assert node.tick() == NodeStatus.SUCCESS

    def test_robot_far_from_goal(self):
        """Robot far from goal → FAILURE."""
        node, fake = self._make_node(goal_x=2.0, goal_y=2.0, tolerance=0.5)

        node.tick()

        msg = _make_pose_msg(x=10.0, y=10.0)
        fake.subscriptions["/amcl_pose"].inject(msg)

        assert node.tick() == NodeStatus.FAILURE

    def test_robot_near_goal_no_message(self):
        """No message received yet → FAILURE."""
        node, _ = self._make_node()
        assert node.tick() == NodeStatus.FAILURE


# ---------------------------------------------------------------------------
# IsGoalUpdatedCondition
# ---------------------------------------------------------------------------

class TestIsGoalUpdatedIntegration:
    """IsGoalUpdatedCondition does not use ROS topics — pure blackboard logic."""

    def _make_node(self, goal_str="goal_a"):
        from bteng_nav2.conditions.nav_status import IsGoalUpdatedCondition

        goal = MagicMock()
        goal.__str__ = lambda s: goal_str
        bb = Blackboard()
        bb.set("goal_pose", goal)
        config = NodeConfig(blackboard=bb, input_ports={"goal_pose": "goal_pose"})
        node = IsGoalUpdatedCondition("goal_updated_test", config=config)
        return node, bb

    def test_goal_updated_first_tick_succeeds(self):
        """First tick always returns SUCCESS (hash changes from None)."""
        node, _ = self._make_node()
        assert node.tick() == NodeStatus.SUCCESS

    def test_goal_updated_same_goal_returns_failure(self):
        """Second tick with same goal returns FAILURE."""
        node, _ = self._make_node()
        node.tick()  # first: SUCCESS
        assert node.tick() == NodeStatus.FAILURE

    def test_goal_updated_changed_goal_returns_success(self):
        """Changing the blackboard value after first tick → SUCCESS on next tick."""
        node, bb = self._make_node("goal_a")
        node.tick()  # stores hash of goal_a

        new_goal = MagicMock()
        new_goal.__str__ = lambda s: "goal_b"
        bb.set("goal_pose", new_goal)

        assert node.tick() == NodeStatus.SUCCESS

    def test_goal_updated_same_goal_twice_fails_both(self):
        """Three ticks with same goal: SUCCESS, FAILURE, FAILURE."""
        node, _ = self._make_node("stable_goal")
        assert node.tick() == NodeStatus.SUCCESS
        assert node.tick() == NodeStatus.FAILURE
        assert node.tick() == NodeStatus.FAILURE
