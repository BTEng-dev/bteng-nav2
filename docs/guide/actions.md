# Action nodes

Action nodes wrap Nav2 ROS 2 action clients. Each node sends a goal to an action server, returns `RUNNING` while the action is in progress, and returns `SUCCESS` or `FAILURE` when the action server responds.

All action nodes inherit from `RosActionNode`. A `ros_node` is **not** required at construction:
`RosBTExecutor` injects itself into every ROS-aware node in the tree when it starts, which is why a
tree built from XML by `bteng-nav2 run` needs no ROS handle anywhere. Pass `ros_node=` yourself only
when you want the node to use a handle other than the executor's.

Every action node's server is settable per node — see
[Retargeting endpoints](endpoints.md):

```xml
<ComputePathToPose name="plan_b" goal="{goal}" path="{path_b}"
                   action_name="/planner_b/compute_path_to_pose" />
```

---

## Navigation actions

### NavigateToPoseNode

Send a single goal pose to the `/navigate_to_pose` action server. The Nav2 stack handles global planning, local control, and recovery internally.

**Ports**

| Name | Direction | Default | Description |
|------|-----------|---------|-------------|
| `goal_pose` | IN | — | `PoseStamped` target pose in the map frame |
| `behavior_tree` | IN | `""` | Path to an XML behavior tree override; empty uses the Nav2 default |
| `result` | OUT | — | `NavigateToPose.Result` written on success |

**Example**

```python
from bteng import Blackboard, NodeConfig, Tree, TreeMetadata
from bteng_ros2 import RosBTExecutor
from bteng_nav2 import NavigateToPoseNode

bb = Blackboard()
bb.set("goal_pose", make_goal("map", x=2.0, y=1.5))

config = NodeConfig(
    blackboard=bb,
    input_ports={"goal_pose": "goal_pose"},
    output_ports={"result": "nav_result"},
)
node = NavigateToPoseNode("navigate", config=config, ros_node=ros_node)
executor = RosBTExecutor(Tree(TreeMetadata(id="navigate"), node), ros_node=ros_node)
status = executor.run(timeout=120.0)
```

---

### NavigateThroughPosesNode

Send an ordered list of `PoseStamped` waypoints to the `/navigate_through_poses` action server. The robot visits each pose in order.

**Ports**

| Name | Direction | Default | Description |
|------|-----------|---------|-------------|
| `poses` | IN | — | `list[PoseStamped]` ordered waypoints |
| `behavior_tree` | IN | `""` | Behavior tree override path |
| `result` | OUT | — | `NavigateThroughPoses.Result` written on success |

**Example**

```python
from bteng_nav2 import NavigateThroughPosesNode

bb.set("waypoints", [make_goal("map", 0.0, 0.0), make_goal("map", 3.0, 0.0)])

config = NodeConfig(
    blackboard=bb,
    input_ports={"poses": "waypoints"},
    output_ports={"result": "patrol_result"},
)
node = NavigateThroughPosesNode("patrol", config=config, ros_node=ros_node)
```

---

## Path planning

### ComputePathToPoseNode

Request a path to a single goal from the Nav2 planner server (`/compute_path_to_pose`). The resulting `nav_msgs/Path` is written to the blackboard for use by `FollowPathNode` or `IsPathValidNode`.

**Ports**

| Name | Direction | Default | Description |
|------|-----------|---------|-------------|
| `goal` | IN | — | `PoseStamped` target pose |
| `start` | IN | `None` | Optional `PoseStamped` start pose; omit to use the robot's current pose |
| `planner_id` | IN | `""` | Planner plugin identifier |
| `path` | OUT | — | `nav_msgs/Path` computed path |

**Example**

```python
from bteng_nav2 import ComputePathToPoseNode

bb.set("goal", make_goal("map", x=3.0, y=-1.0))

config = NodeConfig(
    blackboard=bb,
    input_ports={"goal": "goal"},
    output_ports={"path": "path"},
)
node = ComputePathToPoseNode("compute", config=config, ros_node=ros_node)
```

---

### ComputePathThroughPosesNode

Request a path through multiple waypoints from the Nav2 planner server (`/compute_path_through_poses`).

**Ports**

| Name | Direction | Default | Description |
|------|-----------|---------|-------------|
| `goals` | IN | — | `list[PoseStamped]` ordered waypoints |
| `start` | IN | `None` | Optional start pose |
| `planner_id` | IN | `""` | Planner plugin identifier |
| `path` | OUT | — | `nav_msgs/Path` computed path |

---

### FollowPathNode

Execute a pre-computed path with the Nav2 controller server (`/follow_path`). Pair with `ComputePathToPoseNode` via a shared blackboard key.

**Ports**

| Name | Direction | Default | Description |
|------|-----------|---------|-------------|
| `path` | IN | — | `nav_msgs/Path` to follow |
| `controller_id` | IN | `""` | Controller plugin identifier |
| `goal_checker_id` | IN | `""` | Goal checker plugin identifier |

**Example**

```python
from bteng import SequenceNode, Tree
from bteng_nav2 import ComputePathToPoseNode, FollowPathNode

compute_config = NodeConfig(
    blackboard=bb,
    input_ports={"goal": "goal"},
    output_ports={"path": "path"},
)
compute = ComputePathToPoseNode("compute", config=compute_config, ros_node=ros_node)

follow_config = NodeConfig(
    blackboard=bb,
    input_ports={"path": "path"},
)
follow = FollowPathNode("follow", config=follow_config, ros_node=ros_node)

sequence = SequenceNode("pipeline", children=[compute, follow])
```

---

### SmoothPathNode

Smooth a path with the Nav2 smoother server (`/smooth_path`) before following it.

**Ports**

| Name | Direction | Default | Description |
|------|-----------|---------|-------------|
| `path` | IN | — | `nav_msgs/Path` to smooth |
| `smoother_id` | IN | `""` | Smoother plugin identifier |
| `max_smoothing_duration` | IN | `10.0` | Maximum time budget in seconds |
| `smoothed_path` | OUT | — | `nav_msgs/Path` smoothed result |

---

## Recovery actions

### SpinNode

Rotate the robot in place by `target_yaw` radians (`/spin`). Use as a recovery behaviour after a navigation failure to allow the planner to find a new path.

**Ports**

| Name | Direction | Default | Description |
|------|-----------|---------|-------------|
| `target_yaw` | IN | — | Rotation angle in radians |
| `time_allowance` | IN | `10.0` | Time limit in seconds |

**Example**

```python
from bteng_nav2 import SpinNode

bb.set("target_yaw", 1.57)  # 90 degrees

config = NodeConfig(
    blackboard=bb,
    input_ports={"target_yaw": "target_yaw"},
)
spin = SpinNode("spin", config=config, ros_node=ros_node)
```

---

### BackUpNode

Drive the robot straight back by `backup_dist` metres at `backup_speed` m/s (`/backup`).

**Ports**

| Name | Direction | Default | Description |
|------|-----------|---------|-------------|
| `backup_dist` | IN | `0.15` | Distance to reverse in metres |
| `backup_speed` | IN | `0.025` | Speed in m/s |
| `time_allowance` | IN | `10.0` | Time limit in seconds |

---

### DriveOnHeadingNode

Drive forward along the robot's current heading by `dist_to_travel` metres (`/drive_on_heading`).

**Ports**

| Name | Direction | Default | Description |
|------|-----------|---------|-------------|
| `dist_to_travel` | IN | — | Distance to drive in metres |
| `speed` | IN | — | Forward speed in m/s |
| `time_allowance` | IN | `10.0` | Time limit in seconds |

---

### WaitNode

Block the tree for `wait_duration` seconds before returning `SUCCESS` (`/wait`).

**Ports**

| Name | Direction | Default | Description |
|------|-----------|---------|-------------|
| `wait_duration` | IN | `1` | Duration to wait in seconds |

---

### AssistedTeleopNode

Activate the Nav2 assisted teleop server for `time_allowance` seconds, allowing a human operator to guide the robot while obstacle avoidance remains active (`/assisted_teleop`).

**Ports**

| Name | Direction | Default | Description |
|------|-----------|---------|-------------|
| `time_allowance` | IN | `30.0` | Maximum teleop duration in seconds |

---

## Docking

### DockRobotNode

Send a docking goal to the OpenNav docking server (`/dock_robot`).

**Ports**

| Name | Direction | Default | Description |
|------|-----------|---------|-------------|
| `dock_id` | IN | — | Identifier of the dock to approach |
| `dock_type` | IN | `""` | Dock plugin type; empty uses the server default |
| `success` | OUT | — | Boolean success flag written on completion |

**Example**

```python
from bteng_nav2 import DockRobotNode

bb.set("dock_id", "charger_dock")

config = NodeConfig(
    blackboard=bb,
    input_ports={"dock_id": "dock_id"},
    output_ports={"success": "dock_success"},
)
dock = DockRobotNode("dock", config=config, ros_node=ros_node)
```

---

### UndockRobotNode

Send an undocking goal to the OpenNav docking server (`/undock_robot`).

**Ports**

| Name | Direction | Default | Description |
|------|-----------|---------|-------------|
| `dock_type` | IN | `""` | Dock plugin type to release from |
