# Building trees

bteng-nav2 nodes are standard bteng nodes. They compose with `SequenceNode`, `FallbackNode`, `ParallelNode`, decorators, and any custom nodes you write. This page shows the most common patterns.

---

## Sequence: compute then follow

The compute-and-follow pipeline is the lowest-level navigation pattern. `ComputePathToPoseNode` writes a path to the blackboard; `FollowPathNode` reads it. Both nodes share the same blackboard.

```python
from bteng import Blackboard, ExecutorConfig, NodeConfig, SequenceNode, Tree, TreeMetadata
from bteng_ros2 import RosBTExecutor
from bteng_nav2 import ComputePathToPoseNode, FollowPathNode

bb = Blackboard()
bb.set("goal", make_goal("map", x=3.0, y=-1.0))

compute_config = NodeConfig(
    blackboard=bb,
    input_ports={"goal": "goal"},
    output_ports={"path": "path"},   # writes to bb["path"]
)
compute = ComputePathToPoseNode("compute", config=compute_config, ros_node=ros_node)

follow_config = NodeConfig(
    blackboard=bb,
    input_ports={"path": "path"},    # reads from bb["path"]
)
follow = FollowPathNode("follow", config=follow_config, ros_node=ros_node)

sequence = SequenceNode("pipeline", children=[compute, follow])
tree = Tree(TreeMetadata(id="compute_and_follow"), sequence)
executor = RosBTExecutor(tree, ExecutorConfig(tick_interval=0.05), ros_node=ros_node)
status = executor.run(timeout=120.0)
```

The `SequenceNode` ticks `compute` first. If the planner returns a path the node writes it to `bb["path"]` and returns `SUCCESS`. The sequence then ticks `follow`, which reads the same key. If either child fails the sequence stops immediately.

---

## FallbackNode: navigate with recovery

A `FallbackNode` tries its children in order and returns `SUCCESS` as soon as one succeeds. Use it to wrap a primary navigation attempt with a recovery sequence.

```python
from bteng import FallbackNode, SequenceNode, NodeConfig, Tree
from bteng_nav2 import NavigateToPoseNode, ClearGlobalCostmapNode, BackUpNode, SpinNode

primary_config = NodeConfig(
    blackboard=bb,
    input_ports={"goal_pose": "goal_pose"},
    output_ports={"result": "nav_result"},
)
primary = NavigateToPoseNode("primary", config=primary_config, ros_node=ros_node)

clear_config = NodeConfig(blackboard=bb)
clear = ClearGlobalCostmapNode("clear", config=clear_config, ros_node=ros_node)

backup_config = NodeConfig(
    blackboard=bb,
    input_ports={"backup_dist": "backup_dist"},
)
backup = BackUpNode("backup", config=backup_config, ros_node=ros_node)

spin_config = NodeConfig(
    blackboard=bb,
    input_ports={"target_yaw": "target_yaw"},
)
spin = SpinNode("spin", config=spin_config, ros_node=ros_node)

retry_config = NodeConfig(
    blackboard=bb,
    input_ports={"goal_pose": "goal_pose"},
    output_ports={"result": "retry_result"},
)
retry = NavigateToPoseNode("retry", config=retry_config, ros_node=ros_node)

recovery = SequenceNode("recovery", children=[clear, backup, spin, retry])
root = FallbackNode("root", children=[primary, recovery])
tree = Tree(TreeMetadata(id="navigate_with_recovery"), root)
```

Execution flow:

1. Tick `primary`. If it returns `SUCCESS`, the fallback returns `SUCCESS` and recovery is never run.
2. If `primary` returns `FAILURE`, the fallback ticks `recovery`.
3. `recovery` runs `clear`, `backup`, `spin`, then `retry` in order.
4. If `retry` succeeds, the sequence returns `SUCCESS` and the fallback returns `SUCCESS`.
5. If `retry` also fails, the fallback returns `FAILURE`.

---

## Blackboard wiring between nodes

Every input port maps a port name to a blackboard key. Every output port maps a port name to the key where the result is written. Two nodes share data by reading and writing the same key.

```python
# Node A writes "path"
config_a = NodeConfig(
    blackboard=bb,
    output_ports={"path": "shared_path"},
)

# Node B reads the same key
config_b = NodeConfig(
    blackboard=bb,
    input_ports={"path": "shared_path"},
)
```

There is no explicit wiring step. The blackboard is the shared state. You can inspect any key at runtime with `bb.get("shared_path")` for debugging.

---

## Passing ros_node via RosBTExecutor

Every bteng-nav2 node that calls a ROS 2 service or action requires a live `rclpy.node.Node`. Pass it at construction time:

```python
import rclpy
from rclpy.node import Node

rclpy.init()
ros_node = Node("my_bt_node")

navigate = NavigateToPoseNode("navigate", config=config, ros_node=ros_node)
```

Pass that same node to `RosBTExecutor` as `ros_node=`, so it is spun alongside the executor's own
node and its client callbacks are delivered:

```python
from bteng import ExecutorConfig, TreeMetadata
from bteng_ros2 import RosBTExecutor

tree = Tree(TreeMetadata(id="navigate"), navigate)
executor = RosBTExecutor(tree, ExecutorConfig(tick_interval=0.05), ros_node=ros_node)
status = executor.run(timeout=120.0)
```

Use a single `ros_node` instance across all nodes in one tree. Sharing the node lets the action and
service clients share one executor and callback group without conflicts.

If you have no reason to own a node, drop `ros_node=` entirely: `RosBTExecutor` is itself an
`rclpy.Node` and injects itself into every ROS-aware node of the tree at `set_tree()` time. The
nodes then need no `ros_node=` at construction either.

---

## Adding custom nodes

bteng-nav2 nodes are regular bteng nodes. Mix them with any custom `ActionNode`, `ConditionNode`, or `StatefulActionNode`:

```python
from bteng import ActionNode, NodeStatus

class LogGoalNode(ActionNode):
    @classmethod
    def provided_ports(cls):
        from bteng import InputPort
        return [InputPort("goal_pose")]

    def tick(self):
        goal = self.get_input("goal_pose")
        print(f"Goal: x={goal.pose.position.x}, y={goal.pose.position.y}")
        return NodeStatus.SUCCESS

log_config = NodeConfig(
    blackboard=bb,
    input_ports={"goal_pose": "goal_pose"},
)
log_node = LogGoalNode("log_goal", config=log_config)

sequence = SequenceNode("main", children=[log_node, navigate])
```

Custom nodes that do not call ROS 2 do not need `ros_node`.
