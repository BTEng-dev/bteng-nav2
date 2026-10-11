"""Tests for extended bteng_nav2 condition nodes (robot_state, tf_condition)."""
from __future__ import annotations

import sys

# ---------------------------------------------------------------------------
# Add BatteryState to the already-registered sensor_msgs.msg stub
# (conftest.py registers LaserScan/PointCloud2/Imu/NavSatFix but not BatteryState)
# ---------------------------------------------------------------------------
_sensor_mod = sys.modules.get("sensor_msgs.msg")
if _sensor_mod is not None:
    class _BatteryState:
        def __init__(self):
            self.percentage = 1.0
            self.voltage = 12.0
            self.current = 0.0
            self.charge = float("nan")
            self.capacity = float("nan")
            self.design_capacity = float("nan")
            self.present = True
            self.power_supply_status = 0
            self.power_supply_health = 0
            self.power_supply_technology = 0
            self.location = ""
            self.serial_number = ""

    _sensor_mod.BatteryState = _BatteryState

import types as _types
from unittest.mock import MagicMock, patch

from bteng import Blackboard, NodeConfig, NodeStatus

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def make_node(cls, **bb_data):
    """Construct a node wired to a fresh Blackboard."""
    from bteng_ros2._mixin import RosNodeMixin

    bb = Blackboard()
    for k, v in bb_data.items():
        bb.set(k, v)
    config = NodeConfig(blackboard=bb, input_ports={k: k for k in bb_data})
    kwargs = {"ros_node": MagicMock()} if issubclass(cls, RosNodeMixin) else {}
    return cls("test", config=config, **kwargs)


def _tick_with_msg(node, msg):
    """Set latest_msg and tick, bypassing the subscription creation."""
    node._latest_msg = msg
    # Subscription already 'created' — set sentinel so tick() skips create_subscription
    node._subscription = MagicMock()
    return node.tick()


# ---------------------------------------------------------------------------
# IsBatteryLowCondition tests
# ---------------------------------------------------------------------------

class TestIsBatteryLowCondition:
    def _make(self, low_threshold=0.20):
        from bteng_nav2.conditions.robot_state import IsBatteryLowCondition

        return make_node(IsBatteryLowCondition, low_threshold=low_threshold)

    def _make_msg(self, percentage):
        msg = _sensor_mod.BatteryState()
        msg.percentage = percentage
        return msg

    def test_no_message_returns_failure(self):
        node = self._make()
        # Pre-set topic_type and subscription so tick() goes straight to msg check
        node._subscription = MagicMock()
        from sensor_msgs.msg import BatteryState
        node.topic_type = BatteryState
        node._latest_msg = None
        result = node.tick()
        assert result == NodeStatus.FAILURE

    def test_battery_low_returns_success(self):
        node = self._make(low_threshold=0.20)
        result = _tick_with_msg(node, self._make_msg(0.10))
        assert result == NodeStatus.SUCCESS

    def test_battery_ok_returns_failure(self):
        node = self._make(low_threshold=0.20)
        result = _tick_with_msg(node, self._make_msg(0.80))
        assert result == NodeStatus.FAILURE

    def test_custom_threshold(self):
        node = self._make(low_threshold=0.40)
        result = _tick_with_msg(node, self._make_msg(0.30))
        assert result == NodeStatus.SUCCESS

    def test_evaluate_battery_low(self):
        node = self._make(low_threshold=0.20)
        assert node.evaluate(self._make_msg(0.10)) is True

    def test_evaluate_battery_ok(self):
        node = self._make(low_threshold=0.20)
        assert node.evaluate(self._make_msg(0.80)) is False

    def test_evaluate_at_threshold_boundary(self):
        """percentage == threshold is NOT low (requires strictly less than)."""
        node = self._make(low_threshold=0.20)
        assert node.evaluate(self._make_msg(0.20)) is False

    def test_tick_sets_topic_type_lazily(self):
        from bteng_nav2.conditions.robot_state import IsBatteryLowCondition

        node = make_node(IsBatteryLowCondition)
        assert node.topic_type is None
        with patch("bteng_ros2.RosConditionNode.tick", return_value=NodeStatus.FAILURE):
            node.tick()
        from sensor_msgs.msg import BatteryState
        assert node.topic_type is BatteryState


# ---------------------------------------------------------------------------
# IsEmergencyStopCondition tests
# ---------------------------------------------------------------------------

class TestIsEmergencyStopCondition:
    def _make(self):
        from bteng_nav2.conditions.robot_state import IsEmergencyStopCondition

        return make_node(IsEmergencyStopCondition)

    def _bool_msg(self, data):
        from std_msgs.msg import Bool
        return Bool(data=data)

    def test_no_message_returns_failure(self):
        node = self._make()
        node._subscription = MagicMock()
        from std_msgs.msg import Bool
        node.topic_type = Bool
        node._latest_msg = None
        assert node.tick() == NodeStatus.FAILURE

    def test_estop_active_returns_success(self):
        node = self._make()
        result = _tick_with_msg(node, self._bool_msg(True))
        assert result == NodeStatus.SUCCESS

    def test_estop_inactive_returns_failure(self):
        node = self._make()
        result = _tick_with_msg(node, self._bool_msg(False))
        assert result == NodeStatus.FAILURE

    def test_evaluate_active(self):
        from bteng_nav2.conditions.robot_state import IsEmergencyStopCondition

        node = make_node(IsEmergencyStopCondition)
        assert node.evaluate(self._bool_msg(True)) is True

    def test_evaluate_inactive(self):
        from bteng_nav2.conditions.robot_state import IsEmergencyStopCondition

        node = make_node(IsEmergencyStopCondition)
        assert node.evaluate(self._bool_msg(False)) is False

    def test_tick_sets_topic_type_lazily(self):
        from bteng_nav2.conditions.robot_state import IsEmergencyStopCondition

        node = make_node(IsEmergencyStopCondition)
        assert node.topic_type is None
        with patch("bteng_ros2.RosConditionNode.tick", return_value=NodeStatus.FAILURE):
            node.tick()
        from std_msgs.msg import Bool
        assert node.topic_type is Bool


# ---------------------------------------------------------------------------
# IsRobotAtFrameCondition tests
# ---------------------------------------------------------------------------

class TestIsRobotAtFrameCondition:
    def _make(self, target_frame="map", robot_frame="base_link", tolerance=0.5):
        from bteng_nav2.conditions.tf_condition import IsRobotAtFrameCondition

        return make_node(
            IsRobotAtFrameCondition,
            target_frame=target_frame,
            robot_frame=robot_frame,
            tolerance=tolerance,
        )

    def _make_transform(self, dx, dy):
        return _types.SimpleNamespace(
            transform=_types.SimpleNamespace(
                translation=_types.SimpleNamespace(x=dx, y=dy, z=0.0),
                rotation=_types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
            )
        )

    def test_robot_at_frame_within_tolerance(self):
        node = self._make(tolerance=0.5)
        t = self._make_transform(0.1, 0.1)
        with patch("tf2_ros.Buffer") as MockBuffer, patch("tf2_ros.TransformListener"):
            MockBuffer.return_value.lookup_transform.return_value = t
            result = node.tick()
        assert result == NodeStatus.SUCCESS

    def test_robot_not_at_frame(self):
        node = self._make(tolerance=0.5)
        t = self._make_transform(5.0, 0.0)
        with patch("tf2_ros.Buffer") as MockBuffer, patch("tf2_ros.TransformListener"):
            MockBuffer.return_value.lookup_transform.return_value = t
            result = node.tick()
        assert result == NodeStatus.FAILURE

    def test_tf_exception_returns_failure(self):
        node = self._make(tolerance=0.5)
        with patch("tf2_ros.Buffer") as MockBuffer, patch("tf2_ros.TransformListener"):
            MockBuffer.return_value.lookup_transform.side_effect = Exception("TF error")
            result = node.tick()
        assert result == NodeStatus.FAILURE

    def test_lazy_init_creates_buffer(self):
        node = self._make(tolerance=0.5)
        assert node._tf_buffer is None
        t = self._make_transform(0.1, 0.1)
        with patch("tf2_ros.Buffer") as MockBuffer, patch("tf2_ros.TransformListener"):
            MockBuffer.return_value.lookup_transform.return_value = t
            node.tick()
        assert node._tf_buffer is not None

    def test_robot_exactly_at_tolerance_boundary(self):
        """Distance exactly equal to tolerance should return SUCCESS (<=)."""
        node = self._make(tolerance=0.5)
        t = self._make_transform(0.5, 0.0)
        with patch("tf2_ros.Buffer") as MockBuffer, patch("tf2_ros.TransformListener"):
            MockBuffer.return_value.lookup_transform.return_value = t
            result = node.tick()
        assert result == NodeStatus.SUCCESS

    def test_reuses_buffer_on_second_tick(self):
        """The TF buffer is created once and reused across ticks."""
        node = self._make(tolerance=0.5)
        t = self._make_transform(0.1, 0.1)
        with patch("tf2_ros.Buffer") as MockBuffer, patch("tf2_ros.TransformListener"):
            MockBuffer.return_value.lookup_transform.return_value = t
            node.tick()
            node.tick()
        # Buffer constructor should be called only once
        assert MockBuffer.call_count == 1
