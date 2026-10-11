"""Example 04 – Navigation with recovery fallback.

Demonstrates a robust navigation tree that attempts the primary
NavigateToPoseNode and, on failure, runs a recovery sequence before
retrying navigation.

Tree structure
--------------
FallbackNode                          (try primary first)
├── NavigateToPoseNode (primary)      goal_pose <- "goal_pose"
└── SequenceNode (recovery + retry)
    ├── ClearGlobalCostmapNode        clear stale obstacles
    ├── BackUpNode                    move away from obstacle
    ├── SpinNode                      reorient to find new path
    └── NavigateToPoseNode (retry)    goal_pose <- "goal_pose"

If the retry also fails the FallbackNode returns FAILURE and the program
reports that navigation could not be completed.

Run
---
    python3 examples/04_recovery_tree.py
"""

from __future__ import annotations

import rclpy

from geometry_msgs.msg import PoseStamped

from bteng import (
    Blackboard,
    ExecutorConfig,
    NodeConfig,
    FallbackNode,
    SequenceNode,
    Tree,
    TreeMetadata,
    NodeStatus,
)
from bteng_ros2 import RosBTExecutor

from bteng_nav2 import (
    NavigateToPoseNode,
    ClearGlobalCostmapNode,
    BackUpNode,
    SpinNode,
)


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
        # 1. Shared blackboard: both navigate nodes read the same goal
        #    None of the nodes below takes a ros_node — RosBTExecutor is an
        #    rclpy node and injects itself into all of them.
        # ------------------------------------------------------------------
        bb = Blackboard()
        bb.set("goal_pose", make_goal("map", x=4.0, y=2.0))
        bb.set("backup_dist", 0.3)
        bb.set("target_yaw", 1.57)   # ~90 degrees

        # ------------------------------------------------------------------
        # 2. Primary navigation attempt
        # ------------------------------------------------------------------
        primary_config = NodeConfig(
            blackboard=bb,
            input_ports={"goal_pose": "goal_pose"},
            output_ports={"result": "nav_result"},
        )
        primary_nav = NavigateToPoseNode("primary_nav", config=primary_config)

        # ------------------------------------------------------------------
        # 3. Recovery sequence
        # ------------------------------------------------------------------
        clear_config = NodeConfig(blackboard=bb)
        clear = ClearGlobalCostmapNode("clear_costmap", config=clear_config)

        backup_config = NodeConfig(
            blackboard=bb,
            input_ports={"backup_dist": "backup_dist"},
        )
        backup = BackUpNode("back_up", config=backup_config)

        spin_config = NodeConfig(
            blackboard=bb,
            input_ports={"target_yaw": "target_yaw"},
        )
        spin = SpinNode("spin", config=spin_config)

        retry_config = NodeConfig(
            blackboard=bb,
            input_ports={"goal_pose": "goal_pose"},
            output_ports={"result": "retry_result"},
        )
        retry_nav = NavigateToPoseNode("retry_nav", config=retry_config)

        recovery_seq = SequenceNode(
            "recovery",
            children=[clear, backup, spin, retry_nav],
        )

        # ------------------------------------------------------------------
        # 4. Top-level fallback
        # ------------------------------------------------------------------
        root = FallbackNode("root", children=[primary_nav, recovery_seq])
        tree = Tree(TreeMetadata(id="recovery_tree"), root)
        executor = RosBTExecutor(tree, ExecutorConfig(tick_interval=0.05))

        executor.get_logger().info("Navigating to (4.0, 2.0) with recovery …")
        # run() returns the status the root settled at — FAILURE also if the
        # timeout expires, in which case the tree is halted first.
        status = executor.run(timeout=180.0)

        if status == NodeStatus.SUCCESS:
            executor.get_logger().info("Navigation succeeded.")
        else:
            executor.get_logger().error("Navigation failed after recovery.")

    finally:
        if executor is not None:
            executor.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
