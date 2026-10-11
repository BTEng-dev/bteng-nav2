# Publisher nodes

Publisher nodes publish a single ROS 2 message to a topic and return `SUCCESS` immediately. They are useful for initialising system state before a navigation tree runs — for example, sending an initial pose to AMCL or forwarding a goal pose to a visualisation tool.

Both nodes inherit from `RosTopicMixin` and `ActionNode`. A `ros_node` is **not** required at
construction: `RosBTExecutor` injects itself into every ROS-aware node when it starts.

Both wait for a subscriber to match before publishing, up to `match_timeout` (2.0 s), and publish
once per activation. A publisher created and published to in the same tick puts its message nowhere
— DDS has not matched the subscriber yet — while the node still reports SUCCESS. That is exactly how
an `/initialpose` publication is lost and AMCL never localises.

These two are the exception to [Retargeting endpoints](endpoints.md): action, service and condition
nodes inherit an endpoint port from their base class, but `RosTopicMixin` has no such hook, so a
publisher's `topic` is a class attribute only. `--namespace` still rewrites it; pointing one at a
different topic in the same namespace needs a subclass.

---

## PublishGoalPoseNode

Publish a `PoseStamped` message to `/goal_pose` and return `SUCCESS`.

**Source:** `bteng_nav2.publishers.goal`  
**Topic:** `/goal_pose` (`geometry_msgs/PoseStamped`)

**Ports**

| Name | Direction | Default | Description |
|------|-----------|---------|-------------|
| `goal_pose` | IN | — | `PoseStamped` to publish |

**Example**

```python
from bteng import NodeConfig
from bteng_nav2 import PublishGoalPoseNode

bb.set("goal_pose", make_goal("map", x=2.0, y=1.5))

config = NodeConfig(
    blackboard=bb,
    input_ports={"goal_pose": "goal_pose"},
)
publish_goal = PublishGoalPoseNode("publish_goal", config=config, ros_node=ros_node)
```

Use this in a sequence before `NavigateToPoseNode` to broadcast the goal pose for RViz or other subscribers.

---

## SetInitialPoseNode

Publish a `PoseWithCovarianceStamped` message to `/initialpose` and return `SUCCESS`. The AMCL localisation node subscribes to this topic to set the robot's initial pose on the map.

**Source:** `bteng_nav2.publishers.localization`  
**Topic:** `/initialpose` (`geometry_msgs/PoseWithCovarianceStamped`)

**Ports**

| Name | Direction | Default | Description |
|------|-----------|---------|-------------|
| `initial_pose` | IN | — | `PoseWithCovarianceStamped` to publish |

**Example**

```python
from geometry_msgs.msg import PoseWithCovarianceStamped
from bteng_nav2 import SetInitialPoseNode

initial = PoseWithCovarianceStamped()
initial.header.frame_id = "map"
initial.pose.pose.position.x = 0.0
initial.pose.pose.position.y = 0.0
initial.pose.pose.orientation.w = 1.0
bb.set("initial_pose", initial)

config = NodeConfig(
    blackboard=bb,
    input_ports={"initial_pose": "initial_pose"},
)
set_pose = SetInitialPoseNode("set_initial_pose", config=config, ros_node=ros_node)
```

Call this at the start of a startup tree before any navigation actions, so AMCL has a pose before the first goal is sent.
