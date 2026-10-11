# bteng-nav2

**bteng-nav2 is a Python library of Nav2 behavior tree nodes built on top of bteng and bteng-ros2 — and a command that runs trees, so a project can need no application Python at all.**

It provides ready-made action, service, condition, and publisher nodes that wrap the standard Nav2 ROS 2 interfaces. Drop them into any bteng tree alongside your own custom nodes to build navigation behaviors — patrol routes, recovery sequences, docking procedures — without writing boilerplate action client or service client code.

---

## Run a tree with one command

A params file, a tree, and `bteng-nav2 run`. No Python, no colcon, no workspace:

```bash
bteng-nav2 run trees/ -p params.yaml              # every tree in the directory
bteng-nav2 run trees/ -p params.yaml --dry-run    # parse + build only: a CI gate, no robot
bteng-nav2 run trees/ -p params.yaml --namespace /robot1
bteng-nav2 nodes                                  # every XML tag this package registers
```

`--dry-run` builds every tree, types every seeded value and needs no ROS installation at all, which
makes it usable as a pull-request check. Full reference: [Running from the CLI](guide/cli.md).

---

## Key features

- **One command** — `bteng-nav2 run trees/ -p params.yaml`; a consuming project ships params and XML only
- **13 action nodes** — `NavigateToPose`, `NavigateThroughPoses`, `ComputePathToPose`, `ComputePathThroughPoses`, `FollowPath`, `SmoothPath`, `Spin`, `BackUp`, `DriveOnHeading`, `Wait`, `AssistedTeleop`, `DockRobot`, `UndockRobot`
- **8 service nodes** — `ClearGlobalCostmap`, `ClearLocalCostmap`, `ClearGlobalCostmapAroundRobot`, `ClearLocalCostmapAroundRobot`, `GetGlobalCostmap`, `GetLocalCostmap`, `IsPathValid`, `LoadMap`
- **6 condition nodes** — `IsRobotNearGoal`, `IsLocalizationActive`, `IsObstacleNear`, `IsStuck`, `IsGoalUpdated`, `IsNavigationActive`
- **2 publisher nodes** — `PublishGoalPose`, `SetInitialPose`
- **Retargetable endpoints** — every action, service and condition node takes its server or topic as a port, so a second planner or a per-robot namespace is a tree change, not a subclass ([Retargeting endpoints](guide/endpoints.md))
- Blackboard-based port wiring — all inputs and outputs flow through the bteng blackboard
- Offline-testable — the test suite runs without a ROS 2 installation using the bundled stub layer

---

## Install

```bash
pip install bteng-nav2
```

With ROS 2 sourced in your environment:

```bash
source /opt/ros/humble/setup.bash
pip install bteng-nav2
```

---

## Where to go next

New users: follow the getting-started path in order.

1. [Installation](getting-started/install.md)
2. [Quick Start](getting-started/quickstart.md)

---

## Node overview

| Category | Nodes |
|----------|-------|
| Running trees from the command line | [Running from the CLI](guide/cli.md) |
| Pointing nodes at your stack | [Retargeting Endpoints](guide/endpoints.md) |
| Navigation actions | [Action Nodes](guide/actions.md) |
| Costmap and map services | [Service Nodes](guide/services.md) |
| State conditions | [Condition Nodes](guide/conditions.md) |
| Topic publishers | [Publisher Nodes](guide/publishers.md) |
| Composing trees | [Building Trees](guide/building-trees.md) |
| Offline testing | [Testing Without ROS](guide/testing.md) |

---

## API reference

| Module | Reference |
|--------|-----------|
| `bteng_nav2.actions` | [Actions](reference/actions.md) |
| `bteng_nav2.services` | [Services](reference/services.md) |
| `bteng_nav2.conditions` | [Conditions](reference/conditions.md) |
| `bteng_nav2.publishers` | [Publishers](reference/publishers.md) |
