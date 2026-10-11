"""Example 03 – Patrol loop through a list of waypoints.

Demonstrates continuous patrol behaviour: the robot cycles through a fixed
list of waypoints indefinitely using NavigateThroughPosesNode.  A new set
of poses is written to the blackboard before each tree tick, so the node
always navigates the full route.

To stop the loop send SIGINT (Ctrl-C).

Tree structure
--------------
NavigateThroughPosesNode   poses <- "waypoints"

The tree is re-run in an infinite loop after each successful lap so the
robot keeps patrolling until the user interrupts the process.  A settled
tree has its tick timer cancelled, so every lap gets a fresh executor —
re-calling run() on the old one would only return the recorded status.

Run
---
    python3 examples/03_patrol_loop.py
"""

from __future__ import annotations

import signal
import sys

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped

from bteng import Blackboard, ExecutorConfig, NodeConfig, Tree, TreeMetadata
from bteng_ros2 import RosBTExecutor

from bteng_nav2 import NavigateThroughPosesNode


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def make_pose(frame_id: str, x: float, y: float, w: float = 1.0) -> PoseStamped:
    """Build a PoseStamped with a 2-D position and yaw-only quaternion."""
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.w = w
    return pose


def build_waypoints(frame_id: str = "map") -> list[PoseStamped]:
    """Return the patrol route as an ordered list of PoseStamped messages."""
    return [
        make_pose(frame_id, x=0.0,  y=0.0),
        make_pose(frame_id, x=3.0,  y=0.0),
        make_pose(frame_id, x=3.0,  y=3.0),
        make_pose(frame_id, x=0.0,  y=3.0),
    ]


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    rclpy.init()
    # This example owns its node instead of letting the executor be the node:
    # the tree node outlives every per-lap executor, so its action client must
    # be bound to a node that stays alive for the whole patrol.
    ros_node = Node("bteng_patrol_loop")

    # Graceful shutdown on Ctrl-C
    running = [True]

    def _sigint_handler(sig, frame):
        ros_node.get_logger().info("Patrol interrupted – shutting down.")
        running[0] = False

    signal.signal(signal.SIGINT, _sigint_handler)

    try:
        bb = Blackboard()
        waypoints = build_waypoints()
        bb.set("waypoints", waypoints)

        config = NodeConfig(
            blackboard=bb,
            input_ports={"poses": "waypoints"},
            output_ports={"result": "patrol_result"},
        )
        navigate = NavigateThroughPosesNode(
            "patrol", config=config, ros_node=ros_node
        )

        tree = Tree(TreeMetadata(id="patrol_loop"), navigate)

        lap = 0
        while running[0]:
            lap += 1
            ros_node.get_logger().info(f"Starting patrol lap {lap} …")

            # Refresh waypoints on the blackboard before each run so that
            # timestamps or dynamic changes can be applied here.
            bb.set("waypoints", waypoints)

            # One executor per lap: once a tree settles its tick timer is
            # cancelled, so run() on the previous executor would return the
            # recorded status without sending a new goal.  The tree node keeps
            # ros_node, which the executor spins alongside its own node.
            executor = RosBTExecutor(
                tree,
                ExecutorConfig(tick_interval=0.05),
                node_name=f"patrol_ticker_{lap}",
                ros_node=ros_node,
            )
            try:
                status = executor.run(timeout=300.0)
            finally:
                executor.destroy_node()

            if not running[0]:
                break
            ros_node.get_logger().info(
                f"Lap {lap} complete ({status.value}) – restarting."
            )

    finally:
        ros_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
