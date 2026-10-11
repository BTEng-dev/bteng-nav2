# Running trees from the command line

A project that uses bteng-nav2 needs **no application Python**. The trees are XML, the poses and
tuning values are YAML, and `bteng-nav2 run` supplies the ROS lifecycle, the tick loop, the failure
report and the exit code.

```bash
bteng-nav2 run trees/ -p params.yaml                  # every *.xml in a directory
bteng-nav2 run trees/patrol.xml -p params.yaml        # one tree
bteng-nav2 run trees/ -p params.yaml --dry-run        # build only, no robot
bteng-nav2 nodes                                      # list the XML tags
```

A project layout ends up looking like this — the same shape as a
[bteng-opcua](https://github.com/mdirzpr/BTEng_opcua) project:

| Path | Written by | Purpose |
|---|---|---|
| `params.yaml` | hand | namespace, `use_sim_time`, poses, tuning values |
| `trees/*.xml` | hand | the behavior trees |
| — | — | there is no application Python |

---

## The params file

```yaml
ros:
  namespace: /robot1          # optional, default ""
  node_name: bteng_nav2       # optional, the rclpy node the tree nodes share
  use_sim_time: true          # optional, default false

blackboard:
  goal_pose:    {type: pose, frame_id: map, x: 2.0, y: 1.5, yaw: 0.0}
  initial_pose: {type: pose_cov, frame_id: map, x: 0.0, y: 0.0, yaw: 0.0}
  waypoints:    {type: poses, frame_id: map, points: [[1.0, 2.0], [3.0, 4.0, 1.57]]}
  backup_dist:  0.30          # scalars pass straight through
  dock_id:      "dock_a"
```

Everything under `blackboard:` is seeded before **each** tree runs. That is the whole point of the
file: a BT XML attribute is a string, but `NavigateToPoseNode.goal_pose` wants a
`geometry_msgs/PoseStamped`. The params file builds the message; the tree only references the key.

### Value types

| `type:` | Message | Keys |
|---|---|---|
| `pose` | `geometry_msgs/PoseStamped` | `frame_id="map"`, `x`, `y`, `z`, `yaw` **or** `qx/qy/qz/qw` |
| `pose_cov` | `geometry_msgs/PoseWithCovarianceStamped` | pose keys + `covariance` (36 numbers) |
| `poses` | `list[PoseStamped]` | `frame_id`, `points:` — `[x, y]`, `[x, y, yaw]`, or a pose mapping |
| `path` | `nav_msgs/Path` | `frame_id`, `points:` (same forms) |
| `point` | `geometry_msgs/Point` | `x`, `y`, `z` |
| `quaternion` | `geometry_msgs/Quaternion` | `yaw` **or** `qx/qy/qz/qw` |
| `duration` | `builtin_interfaces/Duration` | `sec`/`nanosec`, or float `seconds` |

Yaw-only orientation: `z = sin(yaw/2)`, `w = cos(yaw/2)`. `header.stamp` is left at zero — there is
no ROS clock when the file is read, and Nav2 reads a zero stamp as "now".

Validation is strict on purpose. A mapping without a `type:`, an unknown `type:`, an unknown key
inside a spec, or a covariance that is not 36 numbers is an error naming the offending blackboard
key. A typo'd `fram_id` must not silently fall back to `map`.

Anything that is not a mapping — numbers, strings, bools, lists of scalars — passes through
unchanged.

---

## The tree files

```xml
<?xml version="1.0" encoding="UTF-8"?>
<BTEng format_version="1.0" main_tree_to_execute="main">
  <Tree ID="main">
    <Fallback name="navigate_with_recovery">

      <NavigateToPose name="attempt" goal_pose="{goal_pose}" result="{nav_result}" />

      <Sequence name="recover_and_retry">
        <ClearGlobalCostmap name="clear" />
        <BackUp name="back_off" backup_dist="{backup_dist}" />
        <NavigateToPose name="retry" goal_pose="{goal_pose}" />
      </Sequence>

    </Fallback>
  </Tree>
</BTEng>
```

`attr="{key}"` is a blackboard reference; `attr="value"` is a static value. Control nodes
(`Sequence`, `Fallback`, `Parallel`, `ReactiveSequence`, …) and decorators (`Inverter`, `Retry`,
`Timeout`, `RateController`, `ForceSuccess`, `ForceFailure`) come from BTEng itself.
`SetBlackboard` and `CheckBlackboard` are registered for you — BTEng ships them but does not
auto-register them as XML tags.

### Tag names

Every node has **two** tags: the short form and the Python class name. `NavigateToPose` and
`NavigateToPoseNode` are the same node; `IsBatteryLow` and `IsBatteryLowCondition` likewise. The
short form makes hand-written XML read like upstream Nav2 BT XML. `bteng-nav2 nodes` lists all of
them, and `--long` adds each node's ROS action/service/topic name.

### Output ports need no declaration

In plain BTEng XML, `path="{path}"` on a node that *writes* `path` would bind as an input unless the
file carries a `<TreeNodesModel>` block. bteng-nav2 seeds the parser's port directions from each
node's own `provided_ports()`, so a `{key}` attribute binds the way the node declared it. A
`<TreeNodesModel>` block in the file still wins if you write one.

---

## `--dry-run` — the CI gate

```bash
bteng-nav2 run trees/ -p params.yaml --dry-run --plain
```

Parses every tree, resolves every tag, validates every seeded value, prints the tree structure and
what each blackboard key would become, and exits non-zero if anything failed. No ROS context is
created, nothing is contacted, nothing ticks — so a broken tree or a mistyped pose fails in a
pipeline instead of on a robot.

It needs **no ROS installation at all**: not a running graph, not `rclpy`, not the message packages.
The params are validated rather than built — an unknown key, a yaw/quaternion conflict, an
unnormalized quaternion or a wrong-length covariance is still caught — so a typed key reports as
`pose (validated, not built)` rather than as a constructed `PoseStamped`. Drop `--dry-run` and the
same file builds real messages, which does require the message packages.

---

## Multi-robot: `--namespace`

```bash
bteng-nav2 run trees/ -p params.yaml --namespace /robot1
```

Every bteng-nav2 node declares an **absolute** ROS name (`/navigate_to_pose`), and absolute names
ignore a node's namespace — so the names are rewritten explicitly at tree-build time:
`/navigate_to_pose` → `/robot1/navigate_to_pose`. Both the node attribute and the matching
`params` entry are rewritten, because a few nodes recompute their service name in `on_start()`.
Relative and already-namespaced names are left alone, so the rewrite is idempotent.

Endpoints bound to a blackboard key (`action_name="{planner_action}"`) are rewritten too — the
value in the blackboard, not just the attribute — so a params file that names a server does not
quietly escape the namespace.

One exception: `SetCostmapParameterNode` and `GetCostmapParameterNode` derive
`/{ros_node_name}/set_parameters` from a port, and namespacing that would be guesswork. Write the
namespaced value into the port instead.

To point one node somewhere else without moving the whole tree, set its endpoint port directly:
`action_name`, `service_name` or `topic_name`. See [Retargeting endpoints](endpoints.md).

---

## Flags

| Flag | Default | Meaning |
|---|---|---|
| `-p`, `--params` | `./params.yaml` if it exists | Params YAML. An explicitly named file that does not exist is an error |
| `--bb KEY=VALUE` | — | Seed or override one blackboard key; repeatable. YAML-typed, so `--bb enabled=false` is a bool and `--bb n=5` is an int. A mapping is built like a params entry: `--bb 'goal_pose={type: pose, x: 3.0}'` |
| `--namespace` | from params | Override `ros.namespace` |
| `--timeout` | `60` | Per-tree timeout in seconds. On expiry the tree is halted and reported FAILURE |
| `--tick-interval` | `0.05` | Tick period in seconds (20 Hz) |
| `--dry-run` | off | Build only |
| `--plain` | off | No colour, no emoji — for log files and CI |
| `--log` | `INFO` | `DEBUG` / `INFO` / `WARNING` |

Exit code is `0` only if **every** tree ended in SUCCESS. One broken tree becomes a FAILURE result
and never aborts the batch, so a directory run always reports on all of its trees.

---

## Output

```
── 04_recovery.xml ──
🌳 04_recovery.xml started (timeout=60.0s)
🧭 ✅ attempt
🎉 04_recovery.xml finished: SUCCESS

=== Summary ===
  ✅ 04_recovery.xml         SUCCESS
```

One line per leaf as it settles, then a summary. Colour and emoji live only in the `run` command —
the node classes log plain `logging` records under `bteng_nav2.*`, so file and CI logs stay clean.
`--plain` turns the decoration off entirely.

---

## Calling it from Python instead

The CLI is a thin wrapper; the same three pieces are importable:

```python
from bteng_nav2.config import load_config
from bteng_nav2.runtime import install_logging, run_trees
from bteng_nav2.xml_integration import load_tree, register_nodes

install_logging("INFO")                       # only the app layer configures logging
exit_code = run_trees(["trees/"], "params.yaml", presets=["backup_dist=0.5"], timeout=120.0)
```

`register_nodes()` + `load_tree(path, blackboard=bb)` is the low-level pair if you want to build the
tree yourself and drive it with your own executor — see [Building Trees](building-trees.md).
