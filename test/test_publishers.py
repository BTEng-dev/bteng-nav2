"""Tests for bteng_nav2 publisher nodes."""
from __future__ import annotations

from unittest.mock import MagicMock

from bteng import Blackboard, NodeConfig, NodeStatus

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def make_node(cls, **bb_data):
    """Construct a publisher node wired to a fresh Blackboard."""
    bb = Blackboard()
    for k, v in bb_data.items():
        bb.set(k, v)
    config = NodeConfig(blackboard=bb, input_ports={k: k for k in bb_data})
    return cls("test", config=config, ros_node=MagicMock())


# ---------------------------------------------------------------------------
# PublishGoalPoseNode
# ---------------------------------------------------------------------------

class TestPublishGoalPoseNode:
    def _make(self, goal_pose=None):
        from bteng_nav2.publishers.goal import PublishGoalPoseNode

        if goal_pose is None:
            goal_pose = MagicMock()
        return make_node(PublishGoalPoseNode, goal_pose=goal_pose)

    def test_tick_creates_publisher_and_publishes(self):
        goal = MagicMock()
        node = self._make(goal_pose=goal)

        mock_pub = MagicMock()
        node.create_publisher = MagicMock(return_value=mock_pub)

        result = node.tick()

        node.create_publisher.assert_called_once()
        mock_pub.publish.assert_called_once_with(goal)
        assert result == NodeStatus.SUCCESS

    def test_tick_returns_success(self):
        node = self._make()
        node.create_publisher = MagicMock(return_value=MagicMock())
        assert node.tick() == NodeStatus.SUCCESS

    def test_second_tick_reuses_publisher_and_does_not_republish(self):
        goal = MagicMock()
        node = self._make(goal_pose=goal)

        mock_pub = MagicMock()
        node.create_publisher = MagicMock(return_value=mock_pub)

        node.tick()
        node.tick()

        # create_publisher should only be called once
        node.create_publisher.assert_called_once()
        # One message per activation, not one per tick: this is a one-shot
        # publish that reports SUCCESS, and a settled node that gets ticked
        # again must not spam the topic. _on_reset() re-arms it.
        assert mock_pub.publish.call_count == 1

    def test_publishes_to_correct_topic(self):

        node = self._make()
        mock_pub = MagicMock()
        node.create_publisher = MagicMock(return_value=mock_pub)

        node.tick()

        call_args = node.create_publisher.call_args
        # Second positional arg is the topic name
        assert call_args[0][1] == "/goal_pose"


# ---------------------------------------------------------------------------
# SetInitialPoseNode
# ---------------------------------------------------------------------------

class TestSetInitialPoseNode:
    def _make(self, initial_pose=None):
        from bteng_nav2.publishers.localization import SetInitialPoseNode

        if initial_pose is None:
            initial_pose = MagicMock()
        return make_node(SetInitialPoseNode, initial_pose=initial_pose)

    def test_tick_creates_publisher_and_publishes(self):
        pose = MagicMock()
        node = self._make(initial_pose=pose)

        mock_pub = MagicMock()
        node.create_publisher = MagicMock(return_value=mock_pub)

        result = node.tick()

        node.create_publisher.assert_called_once()
        mock_pub.publish.assert_called_once_with(pose)
        assert result == NodeStatus.SUCCESS

    def test_tick_returns_success(self):
        node = self._make()
        node.create_publisher = MagicMock(return_value=MagicMock())
        assert node.tick() == NodeStatus.SUCCESS

    def test_second_tick_reuses_publisher_and_does_not_republish(self):
        pose = MagicMock()
        node = self._make(initial_pose=pose)

        mock_pub = MagicMock()
        node.create_publisher = MagicMock(return_value=mock_pub)

        node.tick()
        node.tick()

        node.create_publisher.assert_called_once()
        # One message per activation, not one per tick: this is a one-shot
        # publish that reports SUCCESS, and a settled node that gets ticked
        # again must not spam the topic. _on_reset() re-arms it.
        assert mock_pub.publish.call_count == 1

    def test_publishes_to_correct_topic(self):
        node = self._make()
        mock_pub = MagicMock()
        node.create_publisher = MagicMock(return_value=mock_pub)

        node.tick()

        call_args = node.create_publisher.call_args
        assert call_args[0][1] == "/initialpose"


class TestPublisherWaitsForSubscriber:
    """A publisher created inside tick() has not been matched to any subscriber
    yet — DDS discovery takes tens to hundreds of milliseconds, and a message
    sent before the match is dropped with no error. An initial pose AMCL never
    receives is the difference between a robot that localizes and one that does
    not, so reporting SUCCESS for that publish is reporting a delivery that did
    not happen."""

    @staticmethod
    def _node(cls_, key, **kw):
        from unittest.mock import MagicMock

        from bteng import Blackboard, NodeConfig

        bb = Blackboard()
        bb.set(key, MagicMock())
        node = cls_("pub", config=NodeConfig(blackboard=bb, input_ports={key: key}), **kw)
        return node

    def _pub_with_subscribers(self, count):
        from unittest.mock import MagicMock

        pub = MagicMock()
        pub.get_subscription_count = MagicMock(return_value=count)
        return pub

    def test_runs_until_a_subscriber_appears(self):
        from bteng import NodeStatus

        from bteng_nav2 import SetInitialPoseNode

        node = self._node(SetInitialPoseNode, "initial_pose")
        pub = self._pub_with_subscribers(0)
        node.create_publisher = lambda *a, **kw: pub

        assert node.tick() == NodeStatus.RUNNING
        assert pub.publish.call_count == 0
        assert "waiting for a subscriber" in node.feedback_message

        pub.get_subscription_count.return_value = 1
        assert node.tick() == NodeStatus.SUCCESS
        assert pub.publish.call_count == 1

    def test_publishes_anyway_once_the_match_timeout_elapses(self):
        from bteng import NodeStatus

        from bteng_nav2 import PublishGoalPoseNode

        node = self._node(PublishGoalPoseNode, "goal_pose")
        node.match_timeout = 0.01
        pub = self._pub_with_subscribers(0)
        node.create_publisher = lambda *a, **kw: pub

        assert node.tick() == NodeStatus.RUNNING
        import time

        time.sleep(0.02)
        assert node.tick() == NodeStatus.SUCCESS
        assert pub.publish.call_count == 1
        assert "no subscriber" in node.feedback_message

    def test_match_timeout_zero_publishes_on_the_first_tick(self):
        from bteng import NodeStatus

        from bteng_nav2 import SetInitialPoseNode

        node = self._node(SetInitialPoseNode, "initial_pose")
        node.match_timeout = 0.0
        pub = self._pub_with_subscribers(0)
        node.create_publisher = lambda *a, **kw: pub

        assert node.tick() == NodeStatus.SUCCESS
        assert pub.publish.call_count == 1

    def test_a_publisher_that_cannot_report_matches_is_treated_as_matched(self):
        """FakeRosNode's publisher has no get_subscription_count; an offline test
        must not hang waiting for a match it can never observe."""
        from bteng import NodeStatus

        from bteng_nav2 import SetInitialPoseNode

        node = self._node(SetInitialPoseNode, "initial_pose")
        from unittest.mock import MagicMock

        pub = MagicMock(spec=["publish"])
        node.create_publisher = lambda *a, **kw: pub

        assert node.tick() == NodeStatus.SUCCESS
        assert pub.publish.call_count == 1
