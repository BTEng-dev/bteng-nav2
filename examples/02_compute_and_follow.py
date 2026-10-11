"""Example 02 – Compute a path then follow it.

Demonstrates a two-step pipeline wired through the blackboard:
  1. ComputePathToPoseNode asks the Nav2 planner for a path and writes it
     to blackboard["path"].
  2. FollowPathNode reads blackboard["path"] and executes it with the
     Nav2 controller.

This pattern is useful when you want to inspect or modify the path between
planning and execution (e.g. validate it with IsPathValidNode first).

Tree structure
--------------
SequenceNode
├── ComputePathToPoseNode  goal -> "goal"  |  path -> "path"
└── FollowPathNode         path <- "path"

Run
---
    python3 examples/02_compute_and_follow.py
"""

from __future__ import annotations

import rclpy

from geometry_msgs.msg import PoseStamped

from bteng import Blackboard, ExecutorConfig, NodeConfig, SequenceNode, Tree, TreeMetadata
from bteng_ros2 import RosBTExecutor

from bteng_nav2 import ComputePathToPoseNode, FollowPathNode


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
        bb.set("goal", make_goal("map", x=3.0, y=-1.0))

        # ------------------------------------------------------------------
        # 2. Build each node
        #    ComputePathToPoseNode writes its result to blackboard["path"].
        #    FollowPathNode reads blackboard["path"].
        #    Neither takes a ros_node: the executor injects itself below.
        # ------------------------------------------------------------------
        compute_config = NodeConfig(
            blackboard=bb,
            input_ports={"goal": "goal"},
            output_ports={"path": "path"},
            # "start" has no usable default, so Tree.validate() wants a value:
            # None means "plan from the robot's current pose".
            params={"start": None},
        )
        compute = ComputePathToPoseNode("compute_path", config=compute_config)

        follow_config = NodeConfig(
            blackboard=bb,
            input_ports={"path": "path"},
        )
        follow = FollowPathNode("follow_path", config=follow_config)

        # ------------------------------------------------------------------
        # 3. Wire into a sequence and run
        #    Tree() takes the metadata first, then the root node.
        # ------------------------------------------------------------------
        sequence = SequenceNode("pipeline", children=[compute, follow])
        tree = Tree(TreeMetadata(id="compute_and_follow"), sequence)
        executor = RosBTExecutor(tree, ExecutorConfig(tick_interval=0.05))

        executor.get_logger().info("Planning and following path to (3.0, -1.0) …")
        # run() spins until the sequence settles, then returns its status.
        status = executor.run(timeout=120.0)
        executor.get_logger().info(f"Tree finished: {status.value}")

    finally:
        if executor is not None:
            executor.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
