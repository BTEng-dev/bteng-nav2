"""Tests for bteng_nav2.xml_integration — factory registration and XML loading.

Runs without ROS: test/conftest.py installs the rclpy / message-package mocks
at import time, and the node classes defer their ROS imports to tick time.
"""
from __future__ import annotations

import pytest
from bteng.core.node import ControlNode, DecoratorNode, TreeNode
from bteng.factory.factory import NodeFactory

import bteng_nav2
from bteng_nav2.xml_integration import (
    BTENG_NODES,
    load_tree,
    port_model,
    register_nodes,
    registered_tags,
)

# ---------------------------------------------------------------------------
# Helpers / fixtures
# ---------------------------------------------------------------------------

TREE_XML = """<?xml version="1.0"?>
<BTEng format_version="1.0" main_tree_to_execute="main">
  <Tree ID="main">
    <Sequence name="root">
      <Inverter>
        <IsBatteryLow low_threshold="0.25"/>
      </Inverter>
      <Fallback name="nav_with_recovery">
        <NavigateToPose goal_pose="{goal_pose}" behavior_tree="navigate_w_replanning.xml"/>
        <SpinNode spin_dist="1.57"/>
      </Fallback>
    </Sequence>
  </Tree>
</BTEng>
"""


def _node_classes_in_all() -> list[type]:
    """Every TreeNode subclass exported from bteng_nav2.__all__."""
    classes = []
    for name in bteng_nav2.__all__:
        obj = getattr(bteng_nav2, name)
        if isinstance(obj, type) and issubclass(obj, TreeNode):
            classes.append(obj)
    return classes


@pytest.fixture
def fresh_factory() -> NodeFactory:
    """A private factory (built-ins only) that is not the global singleton."""
    return NodeFactory()


@pytest.fixture
def clean_singleton():
    """Isolate the NodeFactory singleton so registrations do not leak."""
    NodeFactory.reset_instance()
    yield NodeFactory.get_instance()
    NodeFactory.reset_instance()


def _write_tree(tmp_path, xml: str = TREE_XML):
    path = tmp_path / "tree.xml"
    path.write_text(xml)
    return path


# ---------------------------------------------------------------------------
# BTENG_NODES coverage
# ---------------------------------------------------------------------------

class TestBtengNodesTable:
    def test_every_exported_node_class_is_registered(self):
        """Guard: adding a node to __all__ but forgetting BTENG_NODES fails here."""
        listed = {cls for _tag, cls in BTENG_NODES}
        exported = set(_node_classes_in_all())
        missing = sorted(cls.__name__ for cls in exported - listed)
        assert not missing, f"node classes missing from BTENG_NODES: {missing}"

    def test_table_only_contains_exported_node_classes(self):
        exported = set(_node_classes_in_all())
        extra = sorted({cls for _t, cls in BTENG_NODES} - exported)
        assert not extra, f"BTENG_NODES has classes absent from __all__: {extra}"

    def test_no_duplicate_tags(self):
        tags = [tag for tag, _cls in BTENG_NODES]
        duplicates = sorted({t for t in tags if tags.count(t) > 1})
        assert not duplicates, f"duplicate XML tags: {duplicates}"

    def test_short_and_long_tag_resolve_to_same_class(self):
        mapping = dict(BTENG_NODES)
        for cls in _node_classes_in_all():
            name = cls.__name__
            assert mapping[name] is cls
            for suffix in ("Node", "Condition"):
                if name.endswith(suffix) and len(name) > len(suffix):
                    short = name[: -len(suffix)]
                    assert mapping[short] is cls
                    break
            else:  # pragma: no cover - all current classes have a suffix
                pytest.fail(f"{name} has no Node/Condition suffix to strip")

    def test_two_tags_per_class(self):
        assert len(BTENG_NODES) == 2 * len(_node_classes_in_all())

    def test_expected_short_tags_present(self):
        tags = registered_tags()
        for tag in ("NavigateToPose", "IsBatteryLow", "Spin", "ClearGlobalCostmap", "Wait"):
            assert tag in tags

    def test_registered_tags_is_sorted_and_complete(self):
        tags = registered_tags()
        assert tags == sorted(tags)
        assert set(tags) == {tag for tag, _cls in BTENG_NODES}


# ---------------------------------------------------------------------------
# register_nodes()
# ---------------------------------------------------------------------------

class TestRegisterNodes:
    def test_registers_every_tag_with_the_singleton(self, clean_singleton):
        from bteng import NodeConfig

        factory = clean_singleton
        register_nodes()
        for tag, cls in BTENG_NODES:
            assert factory.is_registered(tag), f"{tag} not registered"
            assert type(factory.create_leaf(tag, "probe", NodeConfig())) is cls

    def test_idempotent(self, clean_singleton):
        from bteng import NodeConfig

        factory = clean_singleton
        register_nodes()
        first = sorted(factory.registered_names())
        register_nodes()
        second = sorted(factory.registered_names())
        assert first == second
        assert len(second) == len(set(second))
        for tag, cls in BTENG_NODES:
            assert type(factory.create_leaf(tag, "probe", NodeConfig())) is cls

    def test_blackboard_builtins_become_available(self, fresh_factory):
        assert not fresh_factory.is_registered("SetBlackboard")
        assert not fresh_factory.is_registered("CheckBlackboard")
        register_nodes(fresh_factory)
        assert fresh_factory.is_registered("SetBlackboard")
        assert fresh_factory.is_registered("CheckBlackboard")

    def test_blackboard_builtins_usable_as_xml_tags(self, tmp_path, fresh_factory):
        xml = """<?xml version="1.0"?>
<BTEng format_version="1.0">
  <Tree ID="main">
    <Sequence name="root">
      <SetBlackboard key="counter" value="1"/>
      <CheckBlackboard key="counter"/>
    </Sequence>
  </Tree>
</BTEng>
"""
        from bteng.nodes.leaf.builtins import CheckBlackboard, SetBlackboard

        root = load_tree(_write_tree(tmp_path, xml), factory=fresh_factory)
        children = root.get_children()
        assert isinstance(children[0], SetBlackboard)
        assert isinstance(children[1], CheckBlackboard)

    def test_explicit_factory_does_not_touch_the_singleton(self, clean_singleton, fresh_factory):
        singleton = clean_singleton
        register_nodes(fresh_factory)
        assert fresh_factory.is_registered("NavigateToPose")
        for tag, _cls in BTENG_NODES:
            assert not singleton.is_registered(tag), f"{tag} leaked into the singleton"
        assert not singleton.is_registered("SetBlackboard")


# ---------------------------------------------------------------------------
# load_tree()
# ---------------------------------------------------------------------------

class TestLoadTree:
    def test_builds_expected_node_types(self, tmp_path, fresh_factory):
        from bteng_nav2 import IsBatteryLowCondition, NavigateToPoseNode, SpinNode

        root = load_tree(_write_tree(tmp_path), factory=fresh_factory)

        assert isinstance(root, ControlNode)
        assert type(root).__name__ == "SequenceNode"

        inverter, fallback = root.get_children()
        assert isinstance(inverter, DecoratorNode)
        assert type(inverter).__name__ == "Inverter"
        assert isinstance(inverter.get_children()[0], IsBatteryLowCondition)

        assert type(fallback).__name__ == "FallbackNode"
        nav, spin = fallback.get_children()
        assert isinstance(nav, NavigateToPoseNode)  # short-form tag
        assert isinstance(spin, SpinNode)  # long-form (class name) tag

    def test_blackboard_ref_is_input_port_plain_attr_is_param(self, tmp_path, fresh_factory):
        root = load_tree(_write_tree(tmp_path), factory=fresh_factory)
        nav = root.get_children()[1].get_children()[0]

        assert nav.config.input_ports == {"goal_pose": "goal_pose"}
        assert "goal_pose" not in nav.config.params
        assert nav.config.params["behavior_tree"] == "navigate_w_replanning.xml"
        assert "behavior_tree" not in nav.config.input_ports

    def test_registers_nodes_on_the_supplied_factory(self, tmp_path, fresh_factory):
        assert not fresh_factory.is_registered("NavigateToPose")
        load_tree(_write_tree(tmp_path), factory=fresh_factory)
        assert fresh_factory.is_registered("NavigateToPose")
        assert fresh_factory.is_registered("SpinNode")

    def test_uses_singleton_by_default(self, tmp_path, clean_singleton):
        root = load_tree(_write_tree(tmp_path))
        assert clean_singleton.is_registered("NavigateToPose")
        assert isinstance(root, ControlNode)

    def test_tree_id_selects_named_tree(self, tmp_path, fresh_factory):
        xml = """<?xml version="1.0"?>
<BTEng format_version="1.0" main_tree_to_execute="main">
  <Tree ID="main">
    <Sequence name="main_root">
      <Wait wait_duration="1.0"/>
    </Sequence>
  </Tree>
  <Tree ID="recovery">
    <Fallback name="recovery_root">
      <ClearLocalCostmap/>
    </Fallback>
  </Tree>
</BTEng>
"""
        from bteng_nav2 import ClearLocalCostmapNode

        root = load_tree(_write_tree(tmp_path, xml), tree_id="recovery", factory=fresh_factory)
        assert root.name == "recovery_root"
        assert isinstance(root.get_children()[0], ClearLocalCostmapNode)

    def test_shared_blackboard_is_used(self, tmp_path, fresh_factory):
        from bteng import Blackboard

        bb = Blackboard()
        bb.set("goal_pose", "sentinel")
        root = load_tree(_write_tree(tmp_path), blackboard=bb, factory=fresh_factory)
        nav = root.get_children()[1].get_children()[0]
        assert nav.config.blackboard is bb
        assert nav.get_input("goal_pose") == "sentinel"


# ---------------------------------------------------------------------------
# port_model() — direction of {blackboard} attributes
# ---------------------------------------------------------------------------

COMPUTE_XML = """<?xml version="1.0"?>
<BTEng format_version="1.0" main_tree_to_execute="main">
  <Tree ID="main">
    <Sequence name="root">
      <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
    </Sequence>
  </Tree>
</BTEng>
"""


class TestPortModel:
    def test_output_ports_are_marked_output(self):
        model = port_model()
        assert model["ComputePathToPose"]["path"] == "output_port"
        assert model["NavigateToPose"]["result"] == "output_port"
        assert model["SmoothPath"]["smoothed_path"] == "output_port"
        assert model["ManageLifecycleNodes"]["success"] == "output_port"

    def test_input_ports_are_marked_input(self):
        model = port_model()
        assert model["NavigateToPose"]["goal_pose"] == "input_port"
        assert model["ComputePathToPose"]["goal"] == "input_port"

    def test_covers_short_and_long_tags(self):
        model = port_model()
        for tag, _cls in BTENG_NODES:
            assert tag in model, f"{tag} missing from port_model()"
        assert model["ComputePathToPoseNode"] == model["ComputePathToPose"]


class TestLoadTreePortDirections:
    def test_output_port_lands_in_output_ports(self, tmp_path, fresh_factory):
        """Regression: without a seeded port model, path="{path}" is a silent input."""
        root = load_tree(_write_tree(tmp_path, COMPUTE_XML), factory=fresh_factory)
        node = root.get_children()[0]

        assert node.config.output_ports == {"path": "path"}
        assert node.config.input_ports == {"goal": "goal"}
        assert "path" not in node.config.input_ports
        assert node.config.params["planner_id"] == "GridBased"

    def test_output_write_reaches_the_blackboard(self, tmp_path, fresh_factory):
        from bteng import Blackboard

        bb = Blackboard()
        root = load_tree(_write_tree(tmp_path, COMPUTE_XML), blackboard=bb, factory=fresh_factory)
        node = root.get_children()[0]
        assert node.set_output("path", "computed") is True
        assert node.config.blackboard.get("path") == "computed"

    def test_tree_nodes_model_in_file_overrides_seeded_direction(self, tmp_path, fresh_factory):
        xml = """<?xml version="1.0"?>
<BTEng format_version="1.0" main_tree_to_execute="main">
  <TreeNodesModel>
    <Action ID="ComputePathToPose">
      <input_port name="goal"/>
      <input_port name="path"/>
    </Action>
  </TreeNodesModel>
  <Tree ID="main">
    <Sequence name="root">
      <ComputePathToPose goal="{goal}" path="{path}"/>
    </Sequence>
  </Tree>
</BTEng>
"""
        root = load_tree(_write_tree(tmp_path, xml), factory=fresh_factory)
        node = root.get_children()[0]
        assert node.config.output_ports == {}
        assert node.config.input_ports == {"goal": "goal", "path": "path"}


# ---------------------------------------------------------------------------
# Package re-exports
# ---------------------------------------------------------------------------

class TestPackageReExports:
    def test_symbols_exported(self):
        for name in (
            "BTENG_NODES",
            "register_nodes",
            "load_tree",
            "registered_tags",
            "port_model",
        ):
            assert name in bteng_nav2.__all__
            assert hasattr(bteng_nav2, name)

    def test_same_objects(self):
        assert bteng_nav2.BTENG_NODES is BTENG_NODES
        assert bteng_nav2.register_nodes is register_nodes
        assert bteng_nav2.load_tree is load_tree
        assert bteng_nav2.registered_tags is registered_tags
        assert bteng_nav2.port_model is port_model
