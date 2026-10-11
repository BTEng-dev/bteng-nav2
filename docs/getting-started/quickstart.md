# Quick Start

There are two ways to send a navigation goal with bteng-nav2. Start with the first one — most
projects never need the second.

---

## Without writing Python

Two files and a command. `params.yaml` holds the values, the tree holds the behaviour:

```yaml
# params.yaml
ros:
  namespace: ""
  use_sim_time: false
blackboard:
  goal_pose: {type: pose, frame_id: map, x: 2.0, y: 1.5, yaw: 0.0}
```

```xml
<!-- trees/navigate.xml -->
<BTEng format_version="1.0" main_tree_to_execute="main">
  <Tree ID="main">
    <NavigateToPose name="go" goal_pose="{goal_pose}" result="{nav_result}" />
  </Tree>
</BTEng>
```

```bash
bteng-nav2 run trees/navigate.xml -p params.yaml --dry-run   # builds it, no ROS needed
bteng-nav2 run trees/navigate.xml -p params.yaml             # runs it
```

Poses are built in YAML because an XML attribute is a string and `NavigateToPose` wants a
`geometry_msgs/PoseStamped`; the params loader has typed builders for poses, paths, points,
quaternions and durations. Exit code is 0 only if every tree succeeded, so this drops straight into
CI. See [Running from the CLI](../guide/cli.md).

---

## From Python

The rest of this page is the same goal built by hand — for embedding a tree in a larger application,
or mixing these nodes with your own. The full example is at `examples/01_navigate_to_pose.py`.

---

## 1. Build a goal pose

```python
from geometry_msgs.msg import PoseStamped

def make_goal(frame_id: str, x: float, y: float, w: float = 1.0) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.w = w
    return pose
```

---

## 2. Populate the blackboard

```python
from bteng import Blackboard

bb = Blackboard()
bb.set("goal_pose", make_goal("map", x=2.0, y=1.5))
```

---

## 3. Create the node

```python
from bteng import NodeConfig
from bteng_nav2 import NavigateToPoseNode

config = NodeConfig(
    blackboard=bb,
    input_ports={"goal_pose": "goal_pose"},
    output_ports={"result": "nav_result"},
)
navigate = NavigateToPoseNode("navigate", config=config)
```

`input_ports` maps the node's port name to the blackboard key that holds the value. `output_ports` maps the node's output port name to the key where the result is written.

No `ros_node=` is needed here: the executor injects one at `set_tree()` time (step 4). Pass it at
construction only when the node must use a specific `rclpy.Node`.

---

## 4. Run with RosBTExecutor

```python
import rclpy
from bteng import ExecutorConfig, Tree, TreeMetadata
from bteng_ros2 import RosBTExecutor

rclpy.init()
executor = None

try:
    bb = Blackboard()
    bb.set("goal_pose", make_goal("map", x=2.0, y=1.5))

    config = NodeConfig(
        blackboard=bb,
        input_ports={"goal_pose": "goal_pose"},
        output_ports={"result": "nav_result"},
    )
    navigate = NavigateToPoseNode("navigate", config=config)

    tree = Tree(TreeMetadata(id="quickstart"), navigate)
    executor = RosBTExecutor(tree, ExecutorConfig(tick_interval=0.05))
    status = executor.run(timeout=120.0)
finally:
    if executor is not None:
        executor.destroy_node()
    rclpy.shutdown()
```

`Tree` takes its metadata first and the root node second. `RosBTExecutor` is itself an `rclpy.Node`
and injects itself into every ROS-aware node in the tree, so the nodes need no `ros_node=`; pass
`ros_node=` only when you want the tree to share a node you already own.

`run(timeout=None)` spins until the root returns `SUCCESS` or `FAILURE` and returns that status. The
ticking comes from the timer the executor created in its constructor — `run()` only spins, so
callbacks from action and service clients are served while the tree waits. On timeout the tree is
halted and `FAILURE` is returned. `rclpy.init()` and `rclpy.shutdown()` remain the caller's job.

---

## 5. Or skip the Python entirely

The same goal, as a params file plus a tree, run with one command:

```bash
bteng-nav2 run trees/ -p params.yaml
```

See [Running from the CLI](../guide/cli.md).

---

## What happened

`NavigateToPoseNode` is a `RosActionNode`. On the first tick it reads `goal_pose` from the blackboard, constructs a `NavigateToPose.Goal`, and sends it to the `/navigate_to_pose` action server. On each subsequent tick it checks the action status. When the action server reports success, the node writes the action result to `nav_result` on the blackboard and returns `NodeStatus.SUCCESS`.

---

## Common mistakes

| Mistake | Fix |
|---------|-----|
| Forgetting to source ROS 2 before running | `source /opt/ros/humble/setup.bash` |
| Passing a `PoseStamped` with an empty `frame_id` | Always set `pose.header.frame_id` — Nav2 requires a valid TF frame |
| No Nav2 stack running | Start Nav2 before executing the tree; the action server must be available |
| `Tree(root=node)` | `Tree` takes metadata first: `Tree(TreeMetadata(id="…"), node)` |
| Building a node outside a tree the executor drives | Every bteng-nav2 node needs a live `rclpy.node.Node`; `RosBTExecutor` injects one, otherwise pass `ros_node=` yourself |
| `rclpy.spin(bt)` for a one-shot tree | It never returns after the tree settles — use `bt.run(timeout=…)` |

Next: [Action Nodes](../guide/actions.md).
