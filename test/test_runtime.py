"""Tests for the run layer: tree discovery, namespace rewriting, reporting.

Nothing here needs a ROS graph: the dry-run path never creates a ROS context,
and the live path is exercised through the _build_executor seam.  rclpy itself
comes from the stubs conftest.py installs.
"""

import logging

import pytest
from bteng import ActionNode, NodeStatus, SequenceNode

from bteng_nav2 import runtime

EXAMPLES = "examples/trees"
PARAMS = "examples/config/nav2_params.yaml"


class _Ok(ActionNode):
    def tick(self):
        return NodeStatus.SUCCESS


class _NamedNode(ActionNode):
    """Stands in for a node that talks to an absolute ROS name."""

    action_name = "/navigate_to_pose"

    def tick(self):
        return NodeStatus.SUCCESS


# ── collect_trees ───────────────────────────────────────────────────────────────

def test_collect_trees_expands_a_directory_sorted(tmp_path):
    (tmp_path / "b.xml").write_text("<x/>")
    (tmp_path / "a.xml").write_text("<x/>")
    (tmp_path / "notes.txt").write_text("ignored")
    found = runtime.collect_trees([tmp_path])
    assert [p.name for p in found] == ["a.xml", "b.xml"]


def test_collect_trees_takes_a_file_as_is(tmp_path):
    f = tmp_path / "one.xml"
    f.write_text("<x/>")
    assert runtime.collect_trees([f]) == [f]


def test_collect_trees_accepts_a_bare_path():
    assert [p.name for p in runtime.collect_trees(f"{EXAMPLES}/01_navigate_to_pose.xml")] == [
        "01_navigate_to_pose.xml"
    ]


# ── apply_namespace ─────────────────────────────────────────────────────────────

def test_apply_namespace_prefixes_absolute_names():
    node = _NamedNode("nav")
    runtime.apply_namespace(node, "/robot1")
    assert node.action_name == "/robot1/navigate_to_pose"


def test_apply_namespace_is_idempotent():
    node = _NamedNode("nav")
    runtime.apply_namespace(node, "/robot1")
    runtime.apply_namespace(node, "/robot1")
    assert node.action_name == "/robot1/navigate_to_pose"


def test_apply_namespace_walks_children():
    a, b = _NamedNode("a"), _NamedNode("b")
    runtime.apply_namespace(SequenceNode("root", children=[a, b]), "/r2")
    assert a.action_name == b.action_name == "/r2/navigate_to_pose"


def test_apply_namespace_does_nothing_without_a_namespace():
    node = _NamedNode("nav")
    runtime.apply_namespace(node, "")
    assert node.action_name == "/navigate_to_pose"


def test_apply_namespace_leaves_relative_names_alone():
    node = _NamedNode("nav")
    node.action_name = "navigate_to_pose"
    runtime.apply_namespace(node, "/robot1")
    assert node.action_name == "navigate_to_pose"


def test_apply_namespace_rewrites_params_too():
    """The few nodes that recompute their service name in on_start() read it
    back from params, so the attribute alone is not enough."""
    from bteng import NodeConfig

    node = _NamedNode("svc", config=NodeConfig(params={"service_name": "/lifecycle/manage"}))
    runtime.apply_namespace(node, "/robot1")
    assert node.config.params["service_name"] == "/robot1/lifecycle/manage"


# ── failures / describe / emoji ─────────────────────────────────────────────────

def test_failures_reports_only_failed_nodes():
    bad, good = _Ok("bad"), _Ok("good")
    bad._status, bad._feedback_message = NodeStatus.FAILURE, "server rejected goal"
    good._status, good._feedback_message = NodeStatus.SUCCESS, "connected"
    found = runtime.failures(SequenceNode("root", children=[bad, good]))
    assert found == [("bad", "server rejected goal")]


def test_failures_skips_a_failure_without_a_message():
    node = _Ok("silent")
    node._status = NodeStatus.FAILURE
    assert runtime.failures(node) == []


def test_describe_indents_by_depth():
    inner = SequenceNode("inner", children=[_Ok("leaf")])
    lines = runtime.describe(SequenceNode("outer", children=[inner]))
    assert lines[0].startswith("SequenceNode")
    assert lines[1].startswith("    SequenceNode")
    assert lines[2].startswith("        _Ok")


def test_emoji_falls_back_for_an_unknown_node():
    assert runtime._emoji_for(_Ok("x")) == "•"


def test_install_logging_replaces_handlers():
    runtime.install_logging("DEBUG", pretty=False)
    root = logging.getLogger()
    assert len(root.handlers) == 1
    assert root.level == logging.DEBUG
    runtime.install_logging("INFO", pretty=False)


# ── dry run over the shipped examples ───────────────────────────────────────────

def test_dry_run_builds_every_example_tree(capsys):
    """The CI gate: every example tree parses, every tag resolves, every seeded
    value builds — with no ROS context, no Nav2 stack and no robot."""
    assert runtime.run_trees([EXAMPLES], PARAMS, dry_run=True, pretty=False) == 0
    out = capsys.readouterr().out
    assert "NavigateToPoseNode" in out
    assert "goal_pose" in out


def test_dry_run_reports_a_broken_tree_without_aborting(tmp_path, capsys):
    (tmp_path / "01_good.xml").write_text(
        '<BTEng format_version="1.0" main_tree_to_execute="main">'
        '<Tree ID="main"><NavigateToPose name="go" goal_pose="{goal_pose}"/></Tree></BTEng>'
    )
    (tmp_path / "02_bad.xml").write_text("<BTEng><Tree ID='main'><NoSuchNode/></Tree></BTEng>")
    assert runtime.run_trees([tmp_path], PARAMS, dry_run=True, pretty=False) == 1
    out = capsys.readouterr().out
    assert "01_good.xml" in out and "02_bad.xml" in out


def test_dry_run_namespace_override_reaches_the_nodes(capsys):
    runtime.run_trees(
        [f"{EXAMPLES}/01_navigate_to_pose.xml"], PARAMS,
        dry_run=True, pretty=False, namespace="/robot1",
    )
    # Rebuild the same tree to inspect it: the run only prints structure.
    from bteng_nav2.config import load_config
    cfg = load_config(PARAMS).with_namespace("/robot1")
    root, _bb = runtime.build_tree(
        runtime.collect_trees(f"{EXAMPLES}/01_navigate_to_pose.xml")[0], cfg.blackboard, cfg
    )
    assert root.action_name == "/robot1/navigate_to_pose"


# ── run_trees guard rails ───────────────────────────────────────────────────────

def test_run_trees_with_no_trees_fails():
    assert runtime.run_trees([], PARAMS) == 1


def test_run_trees_with_a_missing_tree_fails(tmp_path):
    assert runtime.run_trees([tmp_path / "nope.xml"], PARAMS) == 1


def test_run_trees_with_a_missing_params_file_fails():
    assert runtime.run_trees([f"{EXAMPLES}/01_navigate_to_pose.xml"], "no_such_params.yaml") == 1


def test_run_trees_with_a_bad_preset_fails():
    assert runtime.run_trees(
        [f"{EXAMPLES}/01_navigate_to_pose.xml"], PARAMS, presets=["oops"], dry_run=True
    ) == 1


# ── the live path, through the _build_executor seam ─────────────────────────────

class _FakeExecutor:
    def __init__(self, status=NodeStatus.SUCCESS):
        self.status = status
        self.halted = False
        self.destroyed = False

    def run(self, timeout=None):
        return self.status

    def halt(self):
        self.halted = True

    def destroy_node(self):
        self.destroyed = True


class _FakeRosNode:
    def __init__(self):
        self.timers = []

    def create_timer(self, period, cb):
        self.timers.append(cb)
        return cb

    def destroy_timer(self, timer):
        self.timers.remove(timer)

    def destroy_node(self):
        pass


@pytest.fixture
def cfg():
    from bteng_nav2.config import load_config
    return load_config(PARAMS)


def test_run_one_returns_the_executor_status(monkeypatch, cfg):
    fake = _FakeExecutor()
    monkeypatch.setattr(runtime, "_build_executor", lambda *a, **kw: fake)
    path = runtime.collect_trees(f"{EXAMPLES}/01_navigate_to_pose.xml")[0]
    ros_node = _FakeRosNode()
    status = runtime.run_one(path, cfg, cfg.blackboard, 5.0, False, ros_node)
    assert status == NodeStatus.SUCCESS
    assert fake.destroyed


def test_run_one_cleans_up_after_a_failing_tree(monkeypatch, cfg):
    fake = _FakeExecutor(NodeStatus.FAILURE)
    monkeypatch.setattr(runtime, "_build_executor", lambda *a, **kw: fake)
    path = runtime.collect_trees(f"{EXAMPLES}/01_navigate_to_pose.xml")[0]
    assert runtime.run_one(path, cfg, cfg.blackboard, 5.0, False, _FakeRosNode()) == (
        NodeStatus.FAILURE
    )
    assert fake.halted and fake.destroyed


def test_run_one_turns_a_build_error_into_failure(monkeypatch, cfg, tmp_path):
    bad = tmp_path / "bad.xml"
    bad.write_text("<BTEng><Tree ID='main'><NoSuchNode/></Tree></BTEng>")
    called = []
    monkeypatch.setattr(runtime, "_build_executor", lambda *a, **kw: called.append(1))
    assert runtime.run_one(bad, cfg, {}, 5.0, False, _FakeRosNode()) == NodeStatus.FAILURE
    assert called == []


def test_run_one_registers_a_reporter_timer_when_pretty(monkeypatch, cfg):
    monkeypatch.setattr(runtime, "_build_executor", lambda *a, **kw: _FakeExecutor())
    ros_node = _FakeRosNode()
    seen = []
    monkeypatch.setattr(ros_node, "create_timer",
                        lambda period, cb: (seen.append(period), cb)[1])
    path = runtime.collect_trees(f"{EXAMPLES}/01_navigate_to_pose.xml")[0]
    runtime.run_one(path, cfg, cfg.blackboard, 5.0, True, ros_node, tick_interval=0.02)
    assert seen == [0.02]


def test_summary_exit_code():
    assert runtime._summary([("a", NodeStatus.SUCCESS)], False) == 0
    assert runtime._summary([("a", NodeStatus.SUCCESS), ("b", NodeStatus.FAILURE)], False) == 1


# ── manage_context / public node builder (Phase A) ──────────────────────────────

def test_run_trees_can_run_inside_a_caller_owned_context(monkeypatch, cfg):
    """A supervisor that already owns a ROS context must be able to run a tree
    without having its context torn down afterwards."""
    import rclpy

    calls = {"init": 0, "shutdown": 0}
    monkeypatch.setattr(rclpy, "init", lambda *a, **kw: calls.__setitem__("init", calls["init"] + 1))
    monkeypatch.setattr(
        rclpy, "shutdown", lambda *a, **kw: calls.__setitem__("shutdown", calls["shutdown"] + 1)
    )
    monkeypatch.setattr(rclpy, "ok", lambda *a, **kw: True)
    monkeypatch.setattr(runtime, "make_ros_node", lambda c: _FakeRosNode())
    monkeypatch.setattr(runtime, "_build_executor", lambda *a, **kw: _FakeExecutor())

    assert runtime.run_trees(
        [f"{EXAMPLES}/01_navigate_to_pose.xml"], PARAMS, pretty=False, manage_context=False
    ) == 0
    assert calls == {"init": 0, "shutdown": 0}


def test_run_trees_owns_the_context_by_default(monkeypatch, cfg):
    import rclpy

    calls = {"init": 0, "shutdown": 0}
    monkeypatch.setattr(rclpy, "init", lambda *a, **kw: calls.__setitem__("init", calls["init"] + 1))
    monkeypatch.setattr(
        rclpy, "shutdown", lambda *a, **kw: calls.__setitem__("shutdown", calls["shutdown"] + 1)
    )
    monkeypatch.setattr(runtime, "make_ros_node", lambda c: _FakeRosNode())
    monkeypatch.setattr(runtime, "_build_executor", lambda *a, **kw: _FakeExecutor())

    runtime.run_trees([f"{EXAMPLES}/01_navigate_to_pose.xml"], PARAMS, pretty=False)
    assert calls == {"init": 1, "shutdown": 1}


def test_manage_context_false_without_a_context_is_an_error(monkeypatch, cfg):
    import rclpy

    monkeypatch.setattr(rclpy, "ok", lambda *a, **kw: False)
    assert runtime.run_trees(
        [f"{EXAMPLES}/01_navigate_to_pose.xml"], PARAMS, pretty=False, manage_context=False
    ) == 1


def test_make_ros_node_is_public_and_the_private_alias_still_works():
    assert runtime.make_ros_node is runtime._make_ros_node
    assert runtime.make_ros_node.__doc__


# ── use_sim_time and the executor seam ──────────────────────────────────────────
# The seam that every test above *replaces* is itself untested: _build_executor
# is where the executor's node name, the caller's ros_node, the tick interval
# and use_sim_time are wired, and a mistake in any of them is invisible offline
# in exactly the way a wrong endpoint was.  These enter it.


@pytest.fixture
def sim_cfg(tmp_path):
    """A params file with the sim clock on, loaded the way the CLI loads one."""
    from bteng_nav2.config import load_config

    params = tmp_path / "sim.yaml"
    params.write_text("ros:\n  use_sim_time: true\nblackboard: {}\n")
    return load_config(params)


@pytest.mark.parametrize("sim", [True, False])
def test_use_sim_time_reaches_the_shared_ros_node(monkeypatch, cfg, sim_cfg, sim):
    """Tree nodes read the clock from this node, so `use_sim_time` has to arrive
    as a parameter override at construction -- afterwards is too late for a node
    that has already latched it."""
    import rclpy.node

    seen = {}
    monkeypatch.setattr(
        rclpy.node, "Node", lambda name, *a, **kw: seen.update(name=name, kwargs=kw)
    )

    runtime.make_ros_node(sim_cfg if sim else cfg)
    overrides = seen["kwargs"].get("parameter_overrides", [])
    assert [(p.name, p.value) for p in overrides] == [("use_sim_time", sim)]


def test_namespace_reaches_the_shared_ros_node(monkeypatch, cfg):
    import rclpy.node

    seen = {}
    monkeypatch.setattr(
        rclpy.node, "Node", lambda name, *a, **kw: seen.update(name=name, kwargs=kw)
    )
    runtime.make_ros_node(cfg.with_namespace("/robot1"))
    assert seen["kwargs"]["namespace"] == "/robot1"


class _RecordingExecutor:
    """Stands in for RosBTExecutor, capturing how runtime constructs it."""

    def __init__(self, tree, config, node_name=None, ros_node=None):
        self.tree, self.config = tree, config
        self.node_name, self.ros_node = node_name, ros_node
        self.parameters = []

    def set_parameters(self, params):
        self.parameters.extend(params)


def _patch_executor(monkeypatch):
    import bteng_ros2

    monkeypatch.setattr(bteng_ros2, "RosBTExecutor", _RecordingExecutor)


def test_build_executor_wires_the_tree_to_the_callers_node(monkeypatch, cfg):
    _patch_executor(monkeypatch)
    ros_node = _FakeRosNode()
    tree = object()
    ex = runtime._build_executor(tree, cfg, ros_node, 0.05)

    assert ex.tree is tree
    assert ex.ros_node is ros_node, "the tree must share the node its clients live on"
    assert ex.node_name == f"{cfg.ros.node_name}_bt"
    assert ex.config.tick_interval == 0.05


def test_build_executor_gives_the_executor_the_sim_clock(monkeypatch, sim_cfg):
    """The executor is a *second* rclpy node with its own parameters. Without
    this the tick timer runs on the system clock while the tree nodes follow
    the sim clock -- every Timeout in every tree then measures wall seconds."""
    _patch_executor(monkeypatch)
    ex = runtime._build_executor(object(), sim_cfg, _FakeRosNode(), 0.05)
    assert [(p.name, p.value) for p in ex.parameters] == [("use_sim_time", True)]


def test_build_executor_leaves_the_system_clock_alone_by_default(monkeypatch, cfg):
    _patch_executor(monkeypatch)
    assert cfg.ros.use_sim_time is False
    ex = runtime._build_executor(object(), cfg, _FakeRosNode(), 0.05)
    assert ex.parameters == [], "use_sim_time must not be set when the params say false"


def test_the_per_tree_timeout_reaches_the_executor(monkeypatch, cfg):
    """--timeout is the only thing standing between a stuck action server and a
    CI job that never ends, so it has to arrive at run()."""
    seen = {}

    class _TimedExecutor(_FakeExecutor):
        def run(self, timeout=None):
            seen["timeout"] = timeout
            return NodeStatus.SUCCESS

    monkeypatch.setattr(runtime, "_build_executor", lambda *a, **kw: _TimedExecutor())
    path = runtime.collect_trees(f"{EXAMPLES}/01_navigate_to_pose.xml")[0]
    runtime.run_one(path, cfg, cfg.blackboard, 12.5, False, _FakeRosNode())
    assert seen["timeout"] == 12.5
