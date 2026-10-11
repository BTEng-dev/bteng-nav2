# Condition nodes

Condition nodes evaluate a runtime state and return `SUCCESS` or `FAILURE` on every tick. In a `SequenceNode` a failing condition short-circuits execution before the action nodes run. In a `FallbackNode` a succeeding condition short-circuits the remaining branches.

---

## IsRobotNearGoalCondition

**Source:** `bteng_nav2.conditions.pose`  
**Topic:** `/amcl_pose` (`geometry_msgs/PoseWithCovarianceStamped`)

Returns `SUCCESS` when the Euclidean distance between the robot's current AMCL pose and `goal_pose` is less than `tolerance` metres. Returns `FAILURE` otherwise.

**Ports**

| Name | Direction | Default | Description |
|------|-----------|---------|-------------|
| `goal_pose` | IN | — | `PoseStamped` target to measure distance to |
| `tolerance` | IN | `"0.5"` | Acceptable distance in metres (string-cast float) |

**Example**

```python
from bteng_nav2 import IsRobotNearGoalCondition

bb.set("goal_pose", make_goal("map", x=2.0, y=1.5))
bb.set("tolerance", 0.3)

config = NodeConfig(
    blackboard=bb,
    input_ports={
        "goal_pose": "goal_pose",
        "tolerance": "tolerance",
    },
)
near_goal = IsRobotNearGoalCondition("near_goal", config=config, ros_node=ros_node)
```

Place this inside a `FallbackNode` before the navigation node. When the robot is already within `tolerance` of the goal the condition returns `SUCCESS` and the fallback skips navigation entirely.

---

## IsLocalizationActiveCondition

**Source:** `bteng_nav2.conditions.pose`  
**Topic:** `/amcl_pose` (`geometry_msgs/PoseWithCovarianceStamped`)

Returns `SUCCESS` when an AMCL pose message has been received within the last `timeout` seconds. Returns `FAILURE` if no message has arrived within that window, indicating that localization may have died or not yet started.

**Ports**

| Name | Direction | Default | Description |
|------|-----------|---------|-------------|
| `timeout` | IN | `"2.0"` | Maximum age in seconds of the last received AMCL pose |

**Example**

```python
from bteng_nav2 import IsLocalizationActiveCondition

bb.set("amcl_timeout", 2.0)

config = NodeConfig(
    blackboard=bb,
    input_ports={"timeout": "amcl_timeout"},
)
loc_check = IsLocalizationActiveCondition("loc_check", config=config, ros_node=ros_node)
```

Use this as the first child of a navigation sequence to fail fast before sending a goal when AMCL is not running.

---

## IsObstacleNearCondition

**Source:** `bteng_nav2.conditions.obstacle`  
**Topic:** `/scan` (`sensor_msgs/LaserScan`)

Returns `SUCCESS` when the closest valid laser scan reading is less than `min_distance` metres away. Returns `FAILURE` when all readings are farther than the threshold or the scan contains no valid ranges.

**Ports**

| Name | Direction | Default | Description |
|------|-----------|---------|-------------|
| `min_distance` | IN | `"0.5"` | Obstacle detection threshold in metres |

**Example**

```python
from bteng_nav2 import IsObstacleNearCondition

bb.set("obstacle_threshold", 0.4)

config = NodeConfig(
    blackboard=bb,
    input_ports={"min_distance": "obstacle_threshold"},
)
obstacle_check = IsObstacleNearCondition("obstacle", config=config, ros_node=ros_node)
```

Place this condition in a reactive fallback around a navigation action to trigger recovery as soon as an obstacle enters the scan range.

---

## IsStuckCondition

**Source:** `bteng_nav2.conditions.obstacle`  
**Topic:** `/odom` (`nav_msgs/Odometry`)

Returns `SUCCESS` when both linear (`vx`) and angular (`wz`) velocity have been near zero for longer than `stuck_timeout` seconds, indicating the robot is not moving. Returns `FAILURE` while the robot is moving or the timeout has not elapsed.

**Ports**

| Name | Direction | Default | Description |
|------|-----------|---------|-------------|
| `stuck_timeout` | IN | `"3.0"` | Time in seconds with near-zero velocity before declaring stuck |

**Example**

```python
from bteng_nav2 import IsStuckCondition

bb.set("stuck_timeout", 5.0)

config = NodeConfig(
    blackboard=bb,
    input_ports={"stuck_timeout": "stuck_timeout"},
)
stuck_check = IsStuckCondition("stuck", config=config, ros_node=ros_node)
```

Use this condition to detect a stalled navigation action and trigger a recovery branch.

---

## IsGoalUpdatedCondition

**Source:** `bteng_nav2.conditions.nav_status`  
**Topic:** none (pure blackboard comparison)

Returns `SUCCESS` when the `goal_pose` blackboard value has changed since the last call, by comparing a hash of the serialised value. Returns `FAILURE` when the goal is unchanged. Does not require a ROS node.

**Ports**

| Name | Direction | Default | Description |
|------|-----------|---------|-------------|
| `goal_pose` | IN | — | `PoseStamped` to watch for changes |

**Example**

```python
from bteng_nav2 import IsGoalUpdatedCondition

config = NodeConfig(
    blackboard=bb,
    input_ports={"goal_pose": "goal_pose"},
)
goal_updated = IsGoalUpdatedCondition("goal_updated", config=config)
```

Use this at the top of a tree that receives goals dynamically. When the goal changes the condition returns `SUCCESS`, triggering a new navigation action. When the goal is the same the condition returns `FAILURE`, short-circuiting repeat goals.

---

## IsNavigationActiveCondition

**Source:** `bteng_nav2.conditions.nav_status`  
**Topic:** none (action server readiness check)

Returns `SUCCESS` when the `/navigate_to_pose` action server is ready to accept goals. Returns `FAILURE` when the server is not yet available.

**Ports**

This node has no ports.

**Example**

```python
from bteng_nav2 import IsNavigationActiveCondition, NavigateToPoseNode
from bteng import SequenceNode

nav_active_config = NodeConfig(blackboard=bb)
nav_active = IsNavigationActiveCondition("nav_active", config=nav_active_config, ros_node=ros_node)

navigate_config = NodeConfig(
    blackboard=bb,
    input_ports={"goal_pose": "goal_pose"},
    output_ports={"result": "nav_result"},
)
navigate = NavigateToPoseNode("navigate", config=navigate_config, ros_node=ros_node)

# Sequence: check server, then send goal
nav_sequence = SequenceNode("nav_sequence", children=[nav_active, navigate])
```
