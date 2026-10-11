# Service nodes

Service nodes wrap Nav2 ROS 2 service clients. Each node sends a request, returns `RUNNING` while waiting for the response, and returns `SUCCESS` or `FAILURE` based on the service response.

All service nodes inherit from `RosServiceNode`. A `ros_node` is **not** required at construction:
`RosBTExecutor` injects itself into every ROS-aware node when it starts. Pass `ros_node=` yourself
only to override that.

Before its first request each node waits for the service server to appear, because DDS discovery has
not finished at the moment a client is created — without that wait every service node fails its
first tick against a real graph. Each node's server is settable per node through a `service_name`
port; see [Retargeting endpoints](endpoints.md).

---

## Costmap management

### ClearGlobalCostmapNode

Clear the entire global costmap (`/global_costmap/clear_entirely_global_costmap`). Use this as the first step in a recovery sequence to remove stale obstacle data.

**Ports**

This node has no ports. It takes no inputs and produces no outputs.

**Example**

```python
from bteng import NodeConfig
from bteng_nav2 import ClearGlobalCostmapNode

config = NodeConfig(blackboard=bb)
clear = ClearGlobalCostmapNode("clear_global", config=config, ros_node=ros_node)
```

---

### ClearLocalCostmapNode

Clear the entire local costmap (`/local_costmap/clear_entirely_local_costmap`). Useful when the robot is stuck due to sensor noise in the local planner's window.

**Ports**

This node has no ports.

---

### ClearGlobalCostmapAroundRobotNode

Clear the global costmap within a radius around the robot's current position (`/global_costmap/clear_around_global_costmap`).

**Ports**

| Name | Direction | Default | Description |
|------|-----------|---------|-------------|
| `reset_distance` | IN | `1.0` | Radius in metres around the robot to clear |

**Example**

```python
from bteng_nav2 import ClearGlobalCostmapAroundRobotNode

bb.set("clear_radius", 2.0)

config = NodeConfig(
    blackboard=bb,
    input_ports={"reset_distance": "clear_radius"},
)
clear = ClearGlobalCostmapAroundRobotNode("clear_around", config=config, ros_node=ros_node)
```

---

### ClearLocalCostmapAroundRobotNode

Clear the local costmap within a radius around the robot's current position (`/local_costmap/clear_around_local_costmap`).

**Ports**

| Name | Direction | Default | Description |
|------|-----------|---------|-------------|
| `reset_distance` | IN | `1.0` | Radius in metres around the robot to clear |

---

### GetGlobalCostmapNode

Retrieve the current global costmap from the costmap server (`/global_costmap/get_costmap`) and write it to the blackboard.

**Ports**

| Name | Direction | Default | Description |
|------|-----------|---------|-------------|
| `costmap` | OUT | — | `nav_msgs/OccupancyGrid` retrieved costmap |

**Example**

```python
from bteng_nav2 import GetGlobalCostmapNode

config = NodeConfig(
    blackboard=bb,
    output_ports={"costmap": "global_map"},
)
get_map = GetGlobalCostmapNode("get_global", config=config, ros_node=ros_node)
```

---

### GetLocalCostmapNode

Retrieve the current local costmap from the costmap server (`/local_costmap/get_costmap`) and write it to the blackboard.

**Ports**

| Name | Direction | Default | Description |
|------|-----------|---------|-------------|
| `costmap` | OUT | — | `nav_msgs/OccupancyGrid` retrieved costmap |

---

## Path validation

### IsPathValidNode

Check whether a given path is collision-free by calling the `/is_path_valid` service. Returns `SUCCESS` if the path is valid and `FAILURE` if the path contains a collision or the service call fails.

**Ports**

| Name | Direction | Default | Description |
|------|-----------|---------|-------------|
| `path` | IN | — | `nav_msgs/Path` to validate |

**Example**

Combine with `ComputePathToPoseNode` in a sequence to validate before following:

```python
from bteng import SequenceNode
from bteng_nav2 import ComputePathToPoseNode, IsPathValidNode, FollowPathNode

compute_config = NodeConfig(
    blackboard=bb,
    input_ports={"goal": "goal"},
    output_ports={"path": "path"},
)
compute = ComputePathToPoseNode("compute", config=compute_config, ros_node=ros_node)

validate_config = NodeConfig(
    blackboard=bb,
    input_ports={"path": "path"},
)
validate = IsPathValidNode("validate", config=validate_config, ros_node=ros_node)

follow_config = NodeConfig(
    blackboard=bb,
    input_ports={"path": "path"},
)
follow = FollowPathNode("follow", config=follow_config, ros_node=ros_node)

pipeline = SequenceNode("pipeline", children=[compute, validate, follow])
```

---

## Map loading

### LoadMapNode

Load a map from a YAML file via the map server (`/map_server/load_map`). Returns `SUCCESS` when the map server confirms the load, `FAILURE` on error.

**Ports**

| Name | Direction | Default | Description |
|------|-----------|---------|-------------|
| `map_url` | IN | — | Absolute path to the map YAML file |
| `result_code` | OUT | — | Integer result code from the map server |

**Example**

```python
from bteng_nav2 import LoadMapNode

bb.set("map_path", "/maps/warehouse.yaml")

config = NodeConfig(
    blackboard=bb,
    input_ports={"map_url": "map_path"},
    output_ports={"result_code": "load_result"},
)
load = LoadMapNode("load_map", config=config, ros_node=ros_node)
```
