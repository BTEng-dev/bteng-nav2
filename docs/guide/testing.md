# Testing without ROS 2

bteng-nav2 ships a `conftest.py` that stubs the entire ROS 2 Python API so the test suite runs on any machine without a ROS installation. This page explains how the stubs work and how to write your own tests.

---

## How conftest.py stubs work

The file at `test/conftest.py` is loaded automatically by pytest before any test module. It calls a series of `_mock_*()` functions that insert fake module objects directly into `sys.modules`. When a bteng-nav2 node later executes `import rclpy` or `from nav2_msgs.action import NavigateToPose`, Python finds the stub module in `sys.modules` rather than attempting to import the real package.

The stub modules provide:

- `rclpy` — `init`, `shutdown`, `Node`, `ActionClient`, `Future`, `QoSProfile`, and executor classes
- `nav2_msgs.action` — goal, result, and feedback stub classes for all Nav2 actions
- `nav2_msgs.srv` — request and response stubs for all Nav2 services
- `geometry_msgs.msg` — `PoseStamped`, `PoseWithCovarianceStamped`, `Twist`, and related types
- `nav_msgs.msg` — `Path`, `Odometry`, `OccupancyGrid`
- `sensor_msgs.msg` — `LaserScan`, `PointCloud2`
- `opennav_docking_msgs.action` — `DockRobot`, `UndockRobot`

All stub classes have the same attribute structure as the real messages but contain no ROS runtime behaviour.

---

## Running pytest offline

With the stubs in place, run the full test suite without sourcing ROS 2:

```bash
# From the repository root
pytest tests/ -v
```

Run a single file:

```bash
pytest tests/test_actions.py -v
pytest tests/test_conditions.py -v
```

Tests that exercise action or service nodes construct nodes directly and call `on_start()` and `make_goal()` or `make_request()` without spinning an executor, because the stub `ActionClient` and `ServiceClient` return dummy futures.

---

## Writing a node test

The recommended pattern is to construct the node, call lifecycle methods directly, and assert on blackboard state.

```python
import pytest
from bteng import Blackboard, NodeConfig

# conftest.py is already loaded by pytest; no import needed

def test_navigate_to_pose_make_goal():
    from bteng_nav2 import NavigateToPoseNode

    bb = Blackboard()

    # Build a stub PoseStamped using the stubbed geometry_msgs
    from geometry_msgs.msg import PoseStamped
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.pose.position.x = 1.0
    pose.pose.position.y = 2.0

    bb.set("goal_pose", pose)

    config = NodeConfig(
        blackboard=bb,
        input_ports={"goal_pose": "goal_pose"},
        output_ports={"result": "nav_result"},
    )

    # ros_node is not called during make_goal(); pass None for unit tests
    node = NavigateToPoseNode("navigate", config=config, ros_node=None)

    goal = node.make_goal()
    assert goal.pose.pose.position.x == 1.0
    assert goal.pose.pose.position.y == 2.0
```

---

## Integration tests with FakeRosNode

For higher-level tests that exercise the full tick loop, bteng-ros2 provides `FakeRosNode`. It behaves like a real `rclpy.node.Node` but does not connect to a DDS network.

```python
from bteng_ros2.testing import FakeRosNode
from bteng import Blackboard, NodeConfig, Tree
from bteng_ros2 import RosBTExecutor
from bteng_nav2 import IsGoalUpdatedCondition

def test_goal_updated_condition():
    ros_node = FakeRosNode("test_node")
    bb = Blackboard()

    from geometry_msgs.msg import PoseStamped
    pose_a = PoseStamped()
    pose_a.pose.position.x = 1.0
    bb.set("goal_pose", pose_a)

    config = NodeConfig(
        blackboard=bb,
        input_ports={"goal_pose": "goal_pose"},
    )
    condition = IsGoalUpdatedCondition("goal_check", config=config)

    from bteng import NodeStatus
    # First tick: goal changed (from nothing to pose_a)
    assert condition.tick() == NodeStatus.SUCCESS

    # Second tick: goal unchanged
    assert condition.tick() == NodeStatus.FAILURE

    # Update the goal
    pose_b = PoseStamped()
    pose_b.pose.position.x = 2.0
    bb.set("goal_pose", pose_b)

    # Third tick: goal changed again
    assert condition.tick() == NodeStatus.SUCCESS
```

`IsGoalUpdatedCondition` is a pure blackboard condition and does not need `ros_node`, making it the simplest node to test in isolation.

---

## Tips

| Scenario | Approach |
|----------|----------|
| Test `make_goal()` logic | Construct node, call `make_goal()`, assert on returned goal fields |
| Test condition logic | Call `tick()` directly and assert on `NodeStatus` |
| Test output port writes | Call `on_success()` or `on_response()` manually, then `bb.get("key")` |
| Test full tree without ROS | Use `FakeRosNode` from bteng-ros2 and a `RosBTExecutor` |
| Add new message stubs | Extend the `_mock_*()` functions in `test/conftest.py` |
