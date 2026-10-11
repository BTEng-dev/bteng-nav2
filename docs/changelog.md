# Changelog

## 0.2.0 — 2026-10-11

### Changed

- **`bteng>=0.3.2`, `bteng-ros2>=0.3.0`**, raised from `0.3.1` / `0.2.3`. Nothing here
  needs the newer code — all 481 tests and all five example trees pass unchanged on
  either — but 0.3.2 and 0.3.0 are the first releases of each requiring Python 3.12, and
  the old floors let `pip install bteng-ros2` resolve to 0.2.4 + bteng 0.3.1 on a
  3.10/3.11 interpreter. CI was doing exactly that: its 3.10 and 3.11 legs tested a
  different engine than its 3.12 leg. The *behaviour* floors are still 0.3.1 and 0.2.3,
  and the docs now say so.
- **`requires-python = ">=3.12"`**, and the CI matrix drops to `["3.12"]` from
  `["3.10", "3.11", "3.12"]`.

  > [!WARNING]
  > **This drops ROS 2 Humble and Iron**, which ship Python 3.10. Jazzy onward only.
  > It is a deliberate trade — consistency with the engine over LTS reach. Humble users
  > pin `bteng-nav2==0.2.4`, which runs on `bteng` 0.3.1.

- `ruff target-version` follows to `py312`.

### Removed

- **`bteng_nav2/py.typed`.** The marker promised verified annotations that nothing
  verifies: CI runs ruff and pytest with no type checker, and neither `bteng` nor
  `bteng-ros2` ships a marker, so every imported symbol resolved to `Any` regardless.
  The annotations are untouched — only the marker and its `package-data` entry are gone.

### Verified

Against `bteng` 0.3.2: 481 passed, 14 skipped (the same 14 as on 0.3.1 — costmap and map
service nodes needing typed messages). `--dry-run` builds and `Tree.validate()`s all five
example trees with output identical to 0.3.1. The two control-node attributes in those
trees, `<Retry num_attempts="2">` and `<Timeout msec="120000">`, are literals, so 0.3.2's
change to bind `{key}` references on control and decorator attributes does not reach them.

## 0.2.4 (2026-07-29)

### Fixed: seven defects the first runs against a real ROS graph produced

Everything in this package had only ever run against stubs. Run against ROS 2
Jazzy with real `rclpy` and real DDS -- stub servers on the Nav2 endpoints, no
simulator, since what was missing was a *wire* -- three defects appeared
immediately, a fourth once the stub grew the endpoints the last two trees
needed, and three more once every one of the 38 node types was exercised. All
seven passed the full offline suite and `--dry-run` first.

**`start` was required, though everything documents it as optional.**
`InputPort("start", default=None)` declares a *required* port: BTEng reads a
`None` default as "this port has no default". So every tree that planned from
the robot's current pose -- the documented behaviour, and what `make_goal()`
implements -- was rejected at executor construction. `02_compute_and_follow.xml`,
shipped in this repo, was one of them. Optional ports now default to `""`.
`goal_pose`/`goal` are two spellings of one port, so both need a default too,
which means validation can no longer require one of them; `on_start()` now
reports that case by name instead.

**`action_result` is the GetResult *response*, not the Result.** Four nodes read
a payload field straight off it and raised on their first live tick:

```
'ComputePathToPose_GetResult_Response' object has no attribute 'path'
```

`bteng-ros2` documents the property as "Result message from the last completed
action" while storing what `get_result_async()` resolved to -- a wrapper around
`status` and `result`. Its own code reads `action_result.status`, which a Result
message does not have. `bteng_nav2.actions._result.action_result_of()` unwraps
it, and accepts a bare Result too, so a change on either side of that boundary
cannot break the nodes again. Affected: `ComputePathToPose`,
`ComputePathThroughPoses`, `SmoothPath`, `DockRobot`. `NavigateToPose` and
`NavigateThroughPoses` were accidentally correct -- the field they wanted is
itself called `result`.

**`--dry-run` did not validate.** It built each tree and never called
`validate_tree()`, which is what the executor does at construction. The one
command whose job is catching a broken tree before the robot could not catch an
unbound port at all -- which is how the first defect above shipped. It now
validates, and reports each error with the tree and port that produced it.

**A literal number in XML arrived as a string, and rclpy aborts the process.**
`<ManageLifecycleNodes command="0"/>` handed the node `"0"`; `<Wait
wait_duration="3"/>`, `"3"`. XML has no types, and every shipped tree and every
offline test bound these ports to the blackboard, where the params file had
already typed them -- so the literal path was never exercised. Putting a string
in a ROS message's numeric field does not raise:

```
nav2_msgs__srv__manage_lifecycle_nodes__request__convert_from_py:
Assertion `PyLong_Check(field)' failed.
```

That is SIGABRT: a core dump mid-run, with no summary, no exit code and nothing
for a supervisor to act on. `05_full_autonomy.xml` did exactly that.
`xml_integration.coerce_literal_params()` now gives every literal the type its
port declares, once for all 38 node types, and a value that will not convert
raises during `--dry-run` naming the node and port instead of reaching a robot.

**A required numeric port had no default to infer a type from.** The coercion
above reads the type from the port's declared default, and a required port has
none: `<DriveOnHeading dist_to_travel="0.5" speed="0.1"/>` still aborted, this
time in `geometry_msgs__msg__point__convert_from_py`. Inferring from the literal
instead would be worse -- a `dock_id` of `"0123"` would silently become an int --
so `target_yaw`, `dist_to_travel`, `speed` and `param_value` now declare
`type_hint=float`, and the coercion reads that first.

**Docking imported a message package that a Nav2 install does not have.**
`No module named 'opennav_docking_msgs'` -- docking began in the standalone
opennav_docking stack and was merged into Nav2, so Jazzy and newer ship
`nav2_msgs.action.DockRobot`. `DockRobotNode` and `UndockRobotNode` now look in
`nav2_msgs` first and fall back to the standalone package, so both kinds of
install work. `DockRobot.Goal` also carries `use_dock_id`, which defaults to
False and makes the server read `dock_pose` instead: a goal that names a dock by
id now says so.

**A field that looks like a string but is a message.** `slam_toolbox` declares
`SaveMap.Request.name` as `std_msgs/String` -- a message -- while
`SerializePoseGraph.Request.filename`, in the same package, is a plain string.
`SaveSlamMapNode` assigned a `str` to the first, which aborts the process:

```
std_msgs__msg__string__convert_from_py:
Assertion `strncmp("std_msgs.msg._string.String", ...) == 0' failed.
```

The test that covered it asserted `req.name == "my_map"` against a `MagicMock` --
the very shape that crashes.

None of these was visible offline for the same reason: a `MagicMock` answers to
every attribute name, so a test fixture cannot tell the difference between the
object a node expects and the one ROS hands it. The tests added with these fixes
use the real message shapes, and each fix was checked by re-introducing the bug.

## 0.2.3 (2026-07-28)

### Fixed: seven endpoints that do not exist on a real Nav2 stack

Verified against `ros-navigation/navigation2` (main). Every one of these nodes
would have failed against any real stack, whatever the discovery logic did —
and no stub test could show it, because the fakes answer to whatever name they
are handed.

| Node | Was | Now |
|---|---|---|
| `ClearGlobalCostmapNode` | `/clear_costmap_global_srv` | `/global_costmap/clear_entirely_global_costmap` |
| `ClearLocalCostmapNode` | `/clear_costmap_local_srv` | `/local_costmap/clear_entirely_local_costmap` |
| `ClearGlobalCostmapAroundRobotNode` | `/clear_global_costmap_around_robot` | `/global_costmap/clear_around_global_costmap` |
| `ClearLocalCostmapAroundRobotNode` | `/clear_local_costmap_around_robot` | `/local_costmap/clear_around_local_costmap` |
| `GetGlobalCostmapNode` | `/get_costmap_global` | `/global_costmap/get_costmap` |
| `GetLocalCostmapNode` | `/get_costmap_local` | `/local_costmap/get_costmap` |
| `BackUpNode` | `/back_up` | `/backup` |

Sources: `nav2_costmap_2d/src/clear_costmap_service.cpp` builds each service as
`"clear_entirely_"`/`"clear_around_"` + the costmap's name, advertised inside
that costmap node's namespace; `nav2_bt_navigator`'s default behaviour tree uses
`global_costmap/clear_entirely_global_costmap`; `nav2_bringup/params/nav2_params.yaml`
registers the reverse behaviour as `backup`.

`ClearGlobalCostmap` opens the recovery branch of nearly every Nav2 tree, so
recovery would have failed on the first real run.

The corrected names are pinned by a test that cites the Nav2 files they came
from, so a future edit cannot quietly undo them. Any of them can still be
overridden per-node from XML via the endpoint port added in 0.2.2.

## 0.2.2 (2026-07-28)

### Every node can be retargeted from XML

Via bteng-ros2 0.2.3, which declares the endpoint as an input port on the base
classes, all 38 nodes here inherit it:

```xml
<ComputePathToPose name="plan_a" action_name="/planner_a/compute_path_to_pose" .../>
<IsObstacleNear    name="rear"   topic_name="/scan_rear" min_distance="0.4"/>
```

Two planners on one robot, a second controller server, or a per-robot namespace
were previously impossible to express in a tree — every node pinned its endpoint
as a class attribute and only `ManageLifecycleNodes` had a port for it. The
default is still the class attribute, so existing trees are unaffected.

Limitations `bteng_nav2_test` reported, now closed. No behaviour that worked
before changes.

### Added

- `run_trees(..., manage_context=False)` runs in the caller's ROS context
  instead of calling `rclpy.init()`/`shutdown()`, so a supervisor that already
  owns a context can run a tree without having its own torn down afterwards.
- `make_ros_node(cfg)` is public — it is the only thing that applies
  `ros.namespace` and `use_sim_time` to the shared node, and callers were
  reaching for the private name. The old `_make_ros_node` still works.
- `ComputePathToPoseNode` accepts `goal_pose` as well as `goal`. NavigateToPose
  spells it `goal_pose`; wiring the two together meant remembering which node
  used which. `goal_pose` wins when both are set.

### Fixed

- `--namespace` silently missed any endpoint bound to a blackboard key. The name
  lives in the blackboard, and `apply_namespace()` only rewrote nodes and their
  params, so `service_name="{svc}"` escaped the rewrite while every other node
  was namespaced. It now rewrites the value under the key too, once, idempotently.
- The executor node did not inherit `use_sim_time`, so its tick timer ran on the
  system clock in a sim-time run while the tree nodes followed the sim clock.

## 0.2.1 (2026-07-28)

Everything here was found by building `bteng_nav2_test`, a validation project
that contains no application Python — the first consumer to use this package the
way its README claims it can be used.

### Fixed

- **`SetInitialPoseNode` and `PublishGoalPoseNode` published into the void.**
  Both created their publisher and published in the same tick. DDS has not
  matched a subscriber that early, so the message was dropped with no error while
  the node reported SUCCESS — an initial pose AMCL never receives is the
  difference between a robot that localizes and one that does not. Both now
  report RUNNING until a subscriber is matched, publish once, then SUCCESS
  (`match_timeout`, default 2 s, then publish anyway; `0` restores the old
  behaviour). Publishing is now once per activation rather than once per tick.
- **Seven condition classes declared their defaults in the wrong argument.**
  `InputPort("low_threshold", "0.20")` puts `"0.20"` in the *description* slot,
  so `port.default` stayed `None`, the XML parser's default-seeding never fired
  and `provided_ports()` advertised a string as documentation. Eight ports fixed.
- **`IsEmergencyStopCondition` declared a `topic_name` port that nothing read.**
  `RosConditionNode` subscribes to the class attribute, so `topic_name="/estop"`
  in XML bound cleanly, passed every structural check and did nothing.
- **`--dry-run` could not run without ROS**, contradicting its own documentation
  as the CI gate. `load_config()` built real `geometry_msgs` objects, so the
  command died at import on any machine without the message packages. Params are
  now *validated* rather than built under `--dry-run` (unknown keys,
  yaw/quaternion conflicts, unnormalized quaternions and wrong-length covariances
  are all still caught), so the gate needs no ROS at all — not a graph, not
  rclpy, not the message packages.

### Changed

- `load_config(path, build_messages=True)` — pass `False` to validate without
  constructing. Typed keys then hold an `UnbuiltMessage` describing what would
  have been built.

### Known limitations found and not yet fixed

- Every node hard-codes its endpoint as a class attribute; only
  `ManageLifecycleNodesNode` exposes it as a port. So two planners on one robot,
  or a `FollowPath` against a second controller server, cannot be expressed in
  XML at all.
- `apply_namespace()` cannot namespace an endpoint bound to a blackboard key: it
  rewrites the attribute and `NodeConfig.params`, never the blackboard.
- Decorator numbers cannot come from YAML — `<Retry num_attempts="{key}">` is
  rejected at build time, since BTEng resolves those into constructor arguments
  before a blackboard exists.

## 0.2.0 (2026-07-28)

Adds the command-line path: a project can now run Nav2 behaviour trees from a
params file and XML, with no application Python.

### Added

- **`bteng-nav2 run trees/ -p params.yaml`** — ROS lifecycle, tick loop, per-leaf
  reporting, failure report and exit code. `--dry-run` builds every tree without
  creating a ROS context (the CI gate), `--namespace` rewrites absolute action and
  service names for multi-robot runs, `--bb KEY=VALUE` overrides seeded values.
- **`bteng-nav2 nodes`** — lists the 76 XML tags (38 node types, each under its
  short form and its class name).
- **`bteng_nav2.xml_integration`** — `register_nodes()`, `load_tree()`,
  `BTENG_NODES`, `port_model()`. `load_tree()` seeds the parser's port directions
  from each class's `provided_ports()`, so `path="{path}"` on a node that *writes*
  `path` binds as an output. Without that the value is silently dropped.
- **`bteng_nav2.config`** — params YAML to typed ROS messages: `pose`, `pose_cov`,
  `poses`, `path`, `point`, `quaternion`, `duration`. Strict validation: a mapping
  with no `type:`, an unknown type, an unknown key or a wrong-length covariance is
  an error naming the blackboard key.
- **`bteng_nav2.runtime`** — `run_trees()`, `run_one()`, `build_tree()`,
  `apply_namespace()`, `failures()`, `install_logging()`.
- Example params file and five example trees under `examples/`.
- `docs/guide/cli.md` documents the whole workflow.

### Fixed

- 13 action nodes dropped `super().on_start()`'s return value, so every one of
  them reported `None` on its first tick — which the tree read as a terminal
  result. All now `return super().on_start()`.
- Every example and doc page called three APIs that did not exist:
  `Tree(root=...)` (metadata is positional #1), `RosBTExecutor(..., ros_node=...)`
  and `executor.run()`. The latter two now exist in bteng-ros2 0.2.x; the first
  was simply wrong.

### Changed

- Requires **bteng >= 0.3.1** and **bteng-ros2 >= 0.2.1**. The bteng-ros2 floor
  matters most: it carries the server-discovery wait, without which every service
  node here fails on its first tick against a real Nav2 stack.

### Known limitation

Never run against a real robot or simulator. The suite stubs rclpy and every
message package, so timer behaviour, action-client callbacks, discovery timing
and `use_sim_time` are unverified in practice.
