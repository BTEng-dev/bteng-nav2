# Full autonomy

**Source file:** `examples/05_full_autonomy.py`

This example demonstrates a production-style tree that handles three real-world concerns before committing to navigation: a localization gate that fails fast when AMCL is not active, a goal proximity check that skips navigation when the robot is already at the destination, and a navigation sequence with a costmap-clear-and-spin recovery branch.

## Code

```python
"""Example 05 – Full autonomy: localization check, goal skip, and recovery.

Demonstrates a production-style tree that handles three common real-world
concerns before committing to navigation:

  1. Localization gate – fail fast if AMCL has not published a pose.
  2. Goal proximity skip – if the robot is already near the goal, succeed
     immediately without sending a navigation goal.
  3. Navigation with recovery – try NavigateToPoseNode; on failure clear
     costmaps and spin before giving up.

Tree structure
--------------
FallbackNode (root)
├── SequenceNode (pre-checks)
│   ├── IsLocalizationActiveCondition   AMCL alive?
│   └── IsRobotNearGoalCondition        already at goal? → SUCCESS skips nav
├── SequenceNode (navigation)
│   ├── IsNavigationActiveCondition     action server ready?
│   └── NavigateToPoseNode              send goal
└── SequenceNode (recovery)
    ├── ClearGlobalCostmapNode          wipe stale obstacles
    └── SpinNode                        reorient for replanning

Evaluation order
----------------
- If AMCL is alive AND the robot is already near the goal, the pre-checks
  sequence returns SUCCESS → the root FallbackNode short-circuits and the
  whole tree succeeds without navigating.
- Otherwise the pre-checks sequence fails (either AMCL is dead, or the
  robot is not near the goal), and the FallbackNode tries the navigation
  sequence.
- If navigation fails, the FallbackNode tries the recovery sequence.

Run
---
    python3 examples/05_full_autonomy.py
"""

from __future__ import annotations

import rclpy

from geometry_msgs.msg import PoseStamped

from bteng import (
    Blackboard,
    ExecutorConfig,
    FallbackNode,
    NodeConfig,
    NodeStatus,
    SequenceNode,
    Tree,
    TreeMetadata,
)
from bteng_ros2 import RosBTExecutor

from bteng_nav2 import (
    ClearGlobalCostmapNode,
    IsLocalizationActiveCondition,
    IsNavigationActiveCondition,
    IsRobotNearGoalCondition,
    NavigateToPoseNode,
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
        # 1. Shared blackboard
        #    No node below takes a ros_node: RosBTExecutor is an rclpy node
        #    and injects itself into every ROS-aware node of the tree.
        # ------------------------------------------------------------------
        bb = Blackboard()
        bb.set("goal_pose", make_goal("map", x=5.0, y=3.0))
        bb.set("goal_tolerance", 0.3)   # metres – passed to IsRobotNearGoal
        bb.set("amcl_timeout", 2.0)     # seconds – passed to IsLocalizationActive
        bb.set("target_yaw", 1.57)      # ~90° spin during recovery

        # ------------------------------------------------------------------
        # 2. Pre-check sequence: localization alive AND near goal already?
        # ------------------------------------------------------------------
        loc_config = NodeConfig(
            blackboard=bb,
            input_ports={"timeout": "amcl_timeout"},
        )
        loc_check = IsLocalizationActiveCondition(
            "localization_active", config=loc_config
        )

        near_goal_config = NodeConfig(
            blackboard=bb,
            input_ports={
                "goal_pose": "goal_pose",
                "tolerance": "goal_tolerance",
            },
        )
        near_goal = IsRobotNearGoalCondition("near_goal", config=near_goal_config)

        pre_checks = SequenceNode("pre_checks", children=[loc_check, near_goal])

        # ------------------------------------------------------------------
        # 3. Navigation sequence: server ready, then navigate
        # ------------------------------------------------------------------
        nav_active_config = NodeConfig(blackboard=bb)
        nav_active = IsNavigationActiveCondition(
            "nav_active", config=nav_active_config
        )

        navigate_config = NodeConfig(
            blackboard=bb,
            input_ports={"goal_pose": "goal_pose"},
            output_ports={"result": "nav_result"},
        )
        navigate = NavigateToPoseNode("navigate", config=navigate_config)

        nav_sequence = SequenceNode(
            "navigation", children=[nav_active, navigate]
        )

        # ------------------------------------------------------------------
        # 4. Recovery sequence: clear costmap then spin
        # ------------------------------------------------------------------
        clear_config = NodeConfig(blackboard=bb)
        clear = ClearGlobalCostmapNode("clear_costmap", config=clear_config)

        spin_config = NodeConfig(
            blackboard=bb,
            input_ports={"target_yaw": "target_yaw"},
        )
        spin = SpinNode("spin", config=spin_config)

        recovery_sequence = SequenceNode(
            "recovery", children=[clear, spin]
        )

        # ------------------------------------------------------------------
        # 5. Root fallback: pre-checks | navigation | recovery
        # ------------------------------------------------------------------
        root = FallbackNode(
            "root",
            children=[pre_checks, nav_sequence, recovery_sequence],
        )

        tree = Tree(TreeMetadata(id="full_autonomy"), root)
        executor = RosBTExecutor(tree, ExecutorConfig(tick_interval=0.05))

        executor.get_logger().info("Running full-autonomy tree …")
        # run() spins until the root settles and returns that status; on
        # timeout it halts the tree and returns FAILURE.
        status = executor.run(timeout=180.0)

        if status == NodeStatus.SUCCESS:
            executor.get_logger().info("Tree succeeded.")
        else:
            executor.get_logger().error("Tree failed – manual intervention required.")

    finally:
        if executor is not None:
            executor.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
```

## How to run

```bash
source /opt/ros/humble/setup.bash
python3 examples/05_full_autonomy.py
```

The tree exits immediately with `SUCCESS` if the robot is already within 0.3 m of `(5.0, 3.0)` and AMCL has recently published a pose. Otherwise it attempts navigation, falling back to a costmap-clear-and-spin recovery if the navigation action fails.
