# Navigate to pose

**Source file:** `examples/01_navigate_to_pose.py`

This example shows the simplest possible bteng-nav2 tree: a single `NavigateToPoseNode` that sends one goal to the Nav2 stack and blocks until navigation succeeds or fails. It demonstrates how to construct a goal pose, wire it through the blackboard, and run the tree with `RosBTExecutor`.

## Code

```python
"""Example 01 – Navigate to a fixed goal pose.

Demonstrates the simplest possible bteng_nav2 tree: a single
NavigateToPoseNode that sends one goal to the Nav2 stack and blocks
until navigation succeeds or fails.

Tree structure
--------------
NavigateToPoseNode
    input:  goal_pose  <- blackboard["goal_pose"]
    output: result     -> blackboard["nav_result"]

Run
---
    python3 examples/01_navigate_to_pose.py
"""

from __future__ import annotations

import rclpy

from geometry_msgs.msg import PoseStamped

from bteng import Blackboard, ExecutorConfig, NodeConfig, Tree, TreeMetadata
from bteng_ros2 import RosBTExecutor

from bteng_nav2 import NavigateToPoseNode


def make_goal(frame_id: str, x: float, y: float, w: float = 1.0) -> PoseStamped:
    """Build a PoseStamped with a 2-D position and yaw-only quaternion."""
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.w = w
    return pose


def main() -> None:
    rclpy.init()
    executor = None

    try:
        # ------------------------------------------------------------------
        # 1. Populate the blackboard
        # ------------------------------------------------------------------
        bb = Blackboard()
        bb.set("goal_pose", make_goal("map", x=2.0, y=1.5))

        # ------------------------------------------------------------------
        # 2. Build the node
        #    No ros_node here: RosBTExecutor is itself an rclpy node and
        #    injects itself into every ROS-aware node of the tree.
        # ------------------------------------------------------------------
        config = NodeConfig(
            blackboard=bb,
            input_ports={"goal_pose": "goal_pose"},
            output_ports={"result": "nav_result"},
        )
        navigate = NavigateToPoseNode("navigate", config=config)

        # ------------------------------------------------------------------
        # 3. Wrap in a tree and run
        #    Tree() takes the metadata first, then the root node.
        # ------------------------------------------------------------------
        tree = Tree(TreeMetadata(id="navigate_to_pose"), navigate)
        executor = RosBTExecutor(tree, ExecutorConfig(tick_interval=0.05))

        executor.get_logger().info("Navigating to (2.0, 1.5) …")
        # run() spins until the root reports SUCCESS/FAILURE; the ticking
        # itself comes from the timer the executor created.
        status = executor.run(timeout=120.0)
        executor.get_logger().info(f"Tree finished: {status.value}")

    finally:
        if executor is not None:
            executor.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
```

## How to run

With ROS 2 sourced and a Nav2 stack running:

```bash
source /opt/ros/humble/setup.bash
python3 examples/01_navigate_to_pose.py
```

The script logs `Navigating to (2.0, 1.5)` when the action is sent and `Tree finished: SUCCESS` (or `FAILURE`) when `NavigateToPoseNode` settles. `run(timeout=120.0)` bounds the wait: on expiry the tree is halted and `FAILURE` is returned rather than hanging forever.

The same tree needs no Python at all — see [Running from the CLI](../guide/cli.md):

```bash
bteng-nav2 run examples/trees/01_navigate_to_pose.xml -p examples/config/nav2_params.yaml
```
