"""Integration tests for bteng_nav2 publisher nodes using FakeRosNode.

Publisher nodes call create_publisher() on the ros_node then publish(msg).
FakeRosNode records every published message in publisher.published — no mocking
of the publisher internals is needed.
"""
from __future__ import annotations

from unittest.mock import MagicMock

from bteng import Blackboard, NodeConfig, NodeStatus
from bteng_ros2.testing import FakeRosNode

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_node(cls, fake: FakeRosNode, **bb_data):
    bb = Blackboard()
    for k, v in bb_data.items():
        bb.set(k, v)
    config = NodeConfig(blackboard=bb, input_ports={k: k for k in bb_data})
    return cls("pub_test", config=config, ros_node=fake)


# ---------------------------------------------------------------------------
# PublishGoalPoseNode
# ---------------------------------------------------------------------------

class TestPublishGoalPoseIntegration:
    """PublishGoalPoseNode publishes to /goal_pose via FakeRosNode."""

    def _make(self, goal_pose=None):
        from bteng_nav2.publishers.goal import PublishGoalPoseNode

        if goal_pose is None:
            goal_pose = MagicMock()
        fake = FakeRosNode()
        node = _make_node(PublishGoalPoseNode, fake, goal_pose=goal_pose)
        return node, fake

    def test_publish_goal_pose_sends_message(self):
        """The goal pose message appears in fake publisher's published list."""
        goal = MagicMock()
        node, fake = self._make(goal_pose=goal)

        node.tick()

        assert "/goal_pose" in fake.publishers
        assert goal in fake.publishers["/goal_pose"].published

    def test_publish_goal_pose_returns_success(self):
        """tick() always returns NodeStatus.SUCCESS."""
        node, _ = self._make()
        assert node.tick() == NodeStatus.SUCCESS

    def test_publisher_reused_on_second_tick(self):
        """The same _FakePublisher object is used on both ticks."""
        goal = MagicMock()
        node, fake = self._make(goal_pose=goal)

        node.tick()
        pub_after_first = fake.publishers.get("/goal_pose")
        node.tick()
        pub_after_second = fake.publishers.get("/goal_pose")

        # FakeRosNode.create_publisher overwrites the dict entry each call, so
        # the node must NOT call create_publisher twice (it uses hasattr guard).
        assert pub_after_first is pub_after_second

    def test_publish_goal_pose_publishes_once_per_activation(self):
        """Two ticks → one message: the publish is a one-shot, re-armed by reset."""
        goal = MagicMock()
        node, fake = self._make(goal_pose=goal)

        node.tick()
        node.tick()

        assert len(fake.publishers["/goal_pose"].published) == 1

        node._on_reset()
        node.tick()
        assert len(fake.publishers["/goal_pose"].published) == 2

    def test_publish_correct_topic(self):
        """Publisher is registered under /goal_pose."""
        node, fake = self._make()
        node.tick()
        assert "/goal_pose" in fake.publishers

    def test_publish_goal_pose_publishes_exact_object(self):
        """The exact goal object placed in the blackboard is published."""
        from geometry_msgs.msg import PoseStamped

        goal = PoseStamped()
        goal.pose.position.x = 3.14
        node, fake = self._make(goal_pose=goal)

        node.tick()

        published = fake.publishers["/goal_pose"].published
        assert len(published) == 1
        assert published[0] is goal


# ---------------------------------------------------------------------------
# SetInitialPoseNode
# ---------------------------------------------------------------------------

class TestSetInitialPoseIntegration:
    """SetInitialPoseNode publishes to /initialpose via FakeRosNode."""

    def _make(self, initial_pose=None):
        from bteng_nav2.publishers.localization import SetInitialPoseNode

        if initial_pose is None:
            initial_pose = MagicMock()
        fake = FakeRosNode()
        node = _make_node(SetInitialPoseNode, fake, initial_pose=initial_pose)
        return node, fake

    def test_set_initial_pose_sends_message(self):
        """The initial pose message appears in the /initialpose publisher."""
        pose = MagicMock()
        node, fake = self._make(initial_pose=pose)

        node.tick()

        assert "/initialpose" in fake.publishers
        assert pose in fake.publishers["/initialpose"].published

    def test_set_initial_pose_returns_success(self):
        """tick() always returns NodeStatus.SUCCESS."""
        node, _ = self._make()
        assert node.tick() == NodeStatus.SUCCESS

    def test_publisher_reused_on_second_tick(self):
        """create_publisher is called only once — second tick reuses existing publisher."""
        pose = MagicMock()
        node, fake = self._make(initial_pose=pose)

        node.tick()
        pub_first = fake.publishers.get("/initialpose")
        node.tick()
        pub_second = fake.publishers.get("/initialpose")

        assert pub_first is pub_second

    def test_set_initial_pose_publishes_once_per_activation(self):
        """Two ticks → one message: the publish is a one-shot, re-armed by reset."""
        pose = MagicMock()
        node, fake = self._make(initial_pose=pose)

        node.tick()
        node.tick()

        assert len(fake.publishers["/initialpose"].published) == 1

        node._on_reset()
        node.tick()
        assert len(fake.publishers["/initialpose"].published) == 2

    def test_publish_correct_topic(self):
        """Publisher is registered under /initialpose."""
        node, fake = self._make()
        node.tick()
        assert "/initialpose" in fake.publishers

    def test_set_initial_pose_publishes_exact_object(self):
        """The exact pose object placed on the blackboard is published."""
        from geometry_msgs.msg import PoseWithCovarianceStamped

        pose = PoseWithCovarianceStamped()
        pose.pose.pose.position.x = 1.0
        node, fake = self._make(initial_pose=pose)

        node.tick()

        published = fake.publishers["/initialpose"].published
        assert len(published) == 1
        assert published[0] is pose
