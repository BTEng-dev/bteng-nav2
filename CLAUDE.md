# bteng_nav2

Nav2 behaviour-tree nodes for the BTEng engine, plus a CLI that runs XML trees from a params file.
Every ROS message import is lazy (inside methods), so the node modules stay importable without the
Nav2 message packages -- keep it that way.

## Layers

```
bteng_nav2/{actions,services,conditions,publishers}/   38 node classes over bteng_ros2 bases
bteng_nav2/xml_integration.py   BTENG_NODES, register_nodes(), load_tree(), port_model()
bteng_nav2/config.py            params YAML -> Nav2Config(ros, blackboard, raw); typed msg builders
bteng_nav2/runtime.py           ROS lifecycle, tick loop, namespace rewrite, reporting, exit code
bteng_nav2/cli.py               bteng-nav2 run | nodes
```

`bteng-nav2 run trees/ -p params.yaml` is the whole point: a consuming project needs **no
application Python**, the same way a bteng_opcua project needs none. Docs: `docs/guide/cli.md`.

## Things that are easy to get wrong

- **Output ports need seeding.** `XMLTreeParser` reads port direction only from a
  `<TreeNodesModel>` block in the XML, defaulting to *input*. Without help,
  `<ComputePathToPose path="{path}"/>` binds `path` as an input and the computed path silently never
  reaches the blackboard. `load_tree()` therefore pre-seeds the parser's private `_port_model` from
  every registered class's `provided_ports()` (`port_model()`); a `<TreeNodesModel>` block in the
  file still wins, because `_load_port_model` runs during `parse_file`.
- **Two tags per node.** Short form (`NavigateToPose`) and class name (`NavigateToPoseNode`), 76
  tags for 38 classes. `test_xml_integration.py` asserts every class in `__all__` is registered --
  that test is the guard against adding a node and forgetting it.
- **Namespaces.** Every node declares an *absolute* ROS name, which ignores a node namespace, so
  `runtime.apply_namespace()` rewrites names at build time (attribute *and* the `params` entry, for
  the nodes that recompute their service name in `on_start`). Exception:
  `Set/GetCostmapParameterNode` derive `/{ros_node_name}/set_parameters` from a port -- not
  rewritable, document it instead of guessing.
- **`on_start` must return a status.** `StatefulActionNode.tick()` *returns* `on_start()`. A node
  that writes `super().on_start()` without `return` reports `None` on its first tick and the tree
  reads that as terminal. Fixed at 13 sites in `actions/` (2026-07-27); the `services/` nodes always
  returned. If you add a node, `return super().on_start()`.
- **Colour and emoji belong in `runtime.py` only.** Node classes log plain `logging` records under
  `bteng_nav2.*`; file and CI logs must stay clean. `--plain` turns decoration off.
- **Endpoint names are checkable, so check them.** Seven defaults named servers that exist nowhere
  in Nav2 (`/clear_costmap_global_srv`, `/back_up`, ...). Nothing offline can catch that: the client
  is built, the goal is built, every stubbed test passes, and the first live tick waits for a server
  that never appears. Fixed before 0.2.0 against the navigation2 sources and pinned by
  `TestEndpointsMatchNav2` in `test/test_services.py`, which cites the upstream file each name comes
  from. Adding a node means finding its real name, not inferring it from the action type --
  `BackUp`'s server is `backup`.
- **Endpoint ports come from the base class.** `bteng-ros2` hooks `__init_subclass__` and appends
  `action_name` / `service_name` / `topic_name` to whatever `provided_ports()` the subclass defines,
  so all 38 inherit it without mentioning it. Do not re-declare one unless you mean to override the
  default. `RosTopicMixin` has no such hook, so the two publisher nodes have no endpoint port --
  `--namespace` still rewrites their `topic`. Docs: `docs/guide/endpoints.md`.

## Tests (no ROS install needed)

`test/conftest.py` stubs rclpy and every message package at import time.

```bash
.venv/bin/python -m pytest test -q          # 481 passed, 14 skipped (2026-07-29, v0.2.0)
.venv/bin/python -m ruff check bteng_nav2 test    # clean over bteng_nav2/
```

`ruff check test` still reports 14 pre-existing E402/E501 in the older test files (`test_actions.py`,
`test_conditions_ext.py`, `test_params.py`, `test_slam.py`, `test_runtime.py`) -- the E402s are the
stub-installation preamble those files run before importing. Package code is clean; keep it that way.

`test_runtime.py::test_dry_run_builds_every_example_tree` is the CI gate: it builds all five shipped
example trees from `examples/config/nav2_params.yaml` with no ROS context. The live path is covered
through the `runtime._build_executor` seam -- patch that, not rclpy.

`test_start_contract.py` walks every exported node and asserts `on_start()` returns a `NodeStatus` --
the 13-site bug had no test at all, because the suite called `on_start()` a dozen times and never
looked at the result. Two halves: a dynamic one (skips the nodes whose `make_goal()` needs a typed
message) and a static one over `inspect.getsource`, which covers all of them unconditionally. Both
were mutation-checked by re-introducing the bug.

`test_runtime.py` also enters `_build_executor` itself rather than only replacing it: the executor's
node name, the caller's `ros_node`, the tick interval, the per-tree timeout reaching `run()`, and
`use_sim_time` reaching *both* rclpy nodes (the shared one at construction, the executor after --
they are separate nodes with separate parameters, and only the second one governs the tick timer).

**First real-ROS runs done (2026-07-29), and they found seven bugs.** There is no ROS on the dev
machine, but there is Docker: `ros:jazzy-ros-base` + `ros-jazzy-nav2-msgs`, real `rclpy`, real DDS,
and a stub server script answering on the Nav2 endpoints. No simulator -- what the offline suite
lacks is a *wire*, not a robot. All five example trees now run green end to end against it, with the
goals checked as they arrived at the servers. What it found:
`start` declared optional but required (`default=None` means *no default* to BTEng, so the port is
mandatory), `action_result` being the GetResult *response* rather than the Result at 4 sites,
`--dry-run` never validating, and -- the worst one -- a literal number in XML reaching a ROS message
as a *string*, which makes rclpy's C extension abort the process rather than raise.
a literal on a *required* numeric port having no declared type to convert from (hence `type_hint`
on the four numeric ports that have no default), and docking importing `opennav_docking_msgs`, which
a Nav2 install does not have -- the messages moved into `nav2_msgs` -- and `SaveSlamMap` assigning a
`str` to a field slam_toolbox declares as `std_msgs/String`. `test_first_live_run.py` pins all seven.

Coverage: **all 38 node types**, from the five example trees plus four purpose-built ones -- the nine conditions checked in *both* directions (sensor values flipped so
each must answer SUCCESS, then FAILURE; an unwired condition returns FAILURE and would otherwise
look correct), SmoothPath, DriveOnHeading, AssistedTeleop, Dock/Undock, ComputePathThroughPoses, the
costmap around-robot and get-costmap services, IsPathValid, and the costmap parameter services.
map load/save, the two slam_toolbox services (`apt install ros-jazzy-slam-toolbox`), `PublishGoalPose`
and `IsGoalUpdated`. `use_sim_time: true` was confirmed on *both* rclpy nodes against a live `/clock`.
Not covered: a real Nav2 stack -- a stub answers correctly by construction, where a real server
rejects goals, aborts them mid-flight and renames things between distributions. That remains the one
thing only a robot or a simulator can tell us.

**A ROS condition cannot pass on its first tick.** `IsLocalizationActive`, `IsRobotNearGoal`,
`IsNavigationActive` and `IsRobotAtFrame` all return FAILURE on the tick that creates their
subscription or client, because DDS discovery has not delivered anything yet; all four pass as soon
as the tree ticks again. A condition must answer SUCCESS/FAILURE immediately, so this is inherent
rather than a defect -- but a tree that guards its first action with one of them behind a bare
`Sequence` fails instantly on a real graph. Put a `Retry` or a reactive loop around it.

**A MagicMock answers to every attribute name.** That single fact is why none of the first three was
visible offline, and it is worth remembering before trusting any fixture here: `MagicMock().path`
and `MagicMock().result.path` both work, so a mock cannot tell a Result from the wrapper around it.
Use a `SimpleNamespace` with the real shape when the shape is what matters.

Timer/spin behaviour and `use_sim_time` are still proven only against stubs and the container --
what the tests pin is that values *reach* the ROS layer. First run on a real stack should be
`bteng-nav2 run examples/trees/ -p examples/config/nav2_params.yaml --dry-run` then the same without
`--dry-run` against `nav2_bringup tb3_simulation_launch.py`.

**Stub-green means little here.** Of the defects found so far, four surfaced while *writing* live
tests and the seventh endpoint came from reading upstream sources -- none from the 413 passing
tests. `mdirzpr/bteng_nav2_test` is the validation project (params + trees + tests, no application
Python); its `make preflight` reports whether its 9 live tests would actually run, since a skipped
test is green and the usual cause is a venv that cannot see the apt-installed `rclpy`.

## Dependencies

`bteng>=0.3.2`, `bteng-ros2>=0.3.0`, `pyyaml`. Both are on `BTEng-dev` `main`.

The *behaviour* floors are a release lower and unchanged: `bteng` 0.3.1 (the engine audit
release: the `tick()` contract, Parallel thresholds, reactive guards) and `bteng-ros2` 0.2.3
(needs `RosBTExecutor.run()`, its `ros_node=` kwarg, the server-discovery wait -- without that
last one every service node here fails on its first tick against a real graph, because DDS
discovery has not finished when the client is made -- and the endpoint ports). The declared
floors sit one release above because 0.3.2 and 0.3.0 are the first of each to require Python
3.12. Do not lower them back to the behaviour floors: `pip install bteng-ros2` then resolves
to 0.2.4 on a 3.10/3.11 interpreter and the package is silently tested against a different
engine than it ships against.

**Python 3.12, so Humble and Iron are out.** Both are Python 3.10. Jazzy onward only. This
was a deliberate call on 2026-08-30 -- consistency with the engine over LTS reach -- not an
oversight. CI's matrix went from `["3.10", "3.11", "3.12"]` to `["3.12"]` in the same change.
Humble users cannot use this package.

**No `py.typed`.** Removed 2026-08-30: CI runs ruff and pytest with no type checker, and
neither `bteng` nor `bteng-ros2` ships a marker, so every imported symbol resolved to `Any`
anyway. Annotations stay; restore the marker when a type checker runs in CI.

One caveat when working from checkouts: an editable install records its version at install time, so
a bumped `pyproject.toml` leaves the metadata stale while the code is current. Anything that gates
on the recorded version then reads a release that is not the one under test. `make preflight` in
`bteng_nav2_test` reports the drift.
