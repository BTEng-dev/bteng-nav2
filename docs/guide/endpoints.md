# Retargeting endpoints

Every node in this package ships with a default ROS endpoint — `NavigateToPoseNode` talks to
`/navigate_to_pose`, `ClearGlobalCostmapNode` to `/global_costmap/clear_entirely_global_costmap`.
Those defaults are what make a tree readable: a tag and a goal, no plumbing.

They are also the two things most likely to be wrong on *your* stack: a second planner, a second
controller, a per-robot namespace, or a distribution that renamed a service. This page is how you
change them without writing Python.

---

## The endpoint port

Action, service and condition nodes each declare an input port carrying their endpoint:

| Node kind | Port | Default comes from |
|---|---|---|
| Action (`RosActionNode`) | `action_name` | the class attribute |
| Service (`RosServiceNode`) | `service_name` | the class attribute |
| Condition (`RosConditionNode`) | `topic_name` | the class attribute |

Say nothing and the node behaves exactly as before. Set the attribute in XML and that node — only
that node — is retargeted:

```xml
<Sequence name="two_planners">
  <ComputePathToPose name="plan_a" goal="{goal}" path="{path_a}"
                     action_name="/planner_a/compute_path_to_pose" />
  <ComputePathToPose name="plan_b" goal="{goal}" path="{path_b}"
                     action_name="/planner_b/compute_path_to_pose" />
</Sequence>

<IsObstacleNear name="rear" topic_name="/scan_rear" min_distance="0.4" />
```

It is an ordinary port, so it can also come from the blackboard, which means from the params file:

```yaml
blackboard:
  planner_action: /planner_b/compute_path_to_pose
```

```xml
<ComputePathToPose name="plan" goal="{goal}" path="{path}"
                   action_name="{planner_action}" />
```

The port is declared by `bteng-ros2` on the base classes, so all 38 node types have it without any
of them mentioning it. A node that declares the port itself keeps its own declaration.

---

## Namespaces

Node endpoints are **absolute** names (`/navigate_to_pose`, leading slash), and an absolute name
ignores the ROS node's namespace. Setting a namespace on the process therefore does nothing at all
to them. `--namespace` rewrites the names at build time instead:

```bash
bteng-nav2 run trees/ -p params.yaml --namespace /robot1
```

`/navigate_to_pose` becomes `/robot1/navigate_to_pose`, and so does every other absolute endpoint in
the tree, including publisher topics and the endpoint values bound through the blackboard. One set
of trees drives any number of robots.

Two exceptions, both worth knowing before you rely on the rewrite:

* **`SetCostmapParameterNode` / `GetCostmapParameterNode`** build `/{ros_node_name}/set_parameters`
  from a port, so there is no absolute attribute to rewrite. Write the namespaced node name into the
  port yourself.
* **Publisher nodes** (`PublishGoalPoseNode`, `SetInitialPoseNode`) are rewritten by `--namespace`
  but have no per-node endpoint port — `RosTopicMixin` is not hooked the way the three base classes
  above are. Retargeting one inside a namespace needs a subclass.

---

## Check the names against your distribution

Nav2 has renamed endpoints between distributions, and a wrong name is invisible until you have a
robot: the client is created, the goal is built, every offline test passes, and the first tick waits
for a server that will never appear.

Seven of this package's defaults were wrong until 0.2.3 — they named servers that exist nowhere in
Nav2:

| Node | Was | Is |
|---|---|---|
| `ClearGlobalCostmapNode` | `/clear_costmap_global_srv` | `/global_costmap/clear_entirely_global_costmap` |
| `ClearLocalCostmapNode` | `/clear_costmap_local_srv` | `/local_costmap/clear_entirely_local_costmap` |
| `ClearGlobalCostmapAroundRobotNode` | `/clear_global_costmap_around_robot` | `/global_costmap/clear_around_global_costmap` |
| `ClearLocalCostmapAroundRobotNode` | `/clear_local_costmap_around_robot` | `/local_costmap/clear_around_local_costmap` |
| `GetGlobalCostmapNode` | `/get_costmap_global` | `/global_costmap/get_costmap` |
| `GetLocalCostmapNode` | `/get_costmap_local` | `/local_costmap/get_costmap` |
| `BackUpNode` | `/back_up` | `/backup` |

The costmap services are namespaced by their costmap because each costmap node creates its own set.
`BackUpNode`'s server is declared `backup` in `nav2_behaviors`; the *action type* is `BackUp`, which
is where the wrong name came from.

Verify against the stack you actually have, not against this table:

```bash
ros2 action list | grep -E 'navigate|compute|follow|spin|backup'
ros2 service list | grep costmap
```

Then fix any mismatch in the tree with the endpoint port — no code change, no release.

---

## Discovery

A client created microseconds ago is not a client that can reach anything: DDS discovery has not
finished. `bteng-ros2` waits for the server to appear, spread across ticks rather than blocking, up
to `discovery_timeout` (5.0 s by default). Without that wait every service node fails its first tick
against a real graph while passing every stubbed test.

If a node times out waiting, the endpoint is the first thing to check — a name nothing serves and a
stack that is not running look identical from inside the node.
