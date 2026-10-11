"""XML and factory integration for bteng_nav2 nodes with BTEng.

This module bridges bteng_nav2's Nav2 BT nodes into BTEng's singleton
``NodeFactory`` and the ``XMLTreeParser``.  Without it a BT XML file cannot
reference any node of this package: the parser resolves unknown tags through
the factory registry, so nothing is loadable until the classes are registered.

BTEng API used
--------------
- ``NodeFactory.get_instance()``   — retrieve the singleton factory/registry
- ``NodeFactory.register(cls, name)`` — register a node class under a name
- ``NodeFactory.is_registered(name)`` — check whether a name is known
- ``XMLTreeParser``                — parse a BTEng XML file into a live tree;
  it automatically looks up custom node types via the factory, so registering
  nodes before parsing is all that is required.  Its port model (which decides
  whether ``attr="{key}"`` is an input or an output) is seeded here from the
  node classes themselves — see :func:`port_model`.

Tag naming
----------
Every node class contributes **two** XML tags: the short form with the
trailing ``Node``/``Condition`` suffix removed (``NavigateToPose``,
``IsBatteryLow``) and the full class name (``NavigateToPoseNode``).  The short
form is the primary spelling so hand-written XML reads like upstream Nav2 BT
XML, while the class name keeps working for anything that generates tags from
Python identifiers.  Both tags map to the same class.

This module deliberately imports no ``rclpy`` and no ROS message package at
module scope — the node classes defer those imports to tick time — so
registration and XML parsing stay usable on a machine without a live ROS
installation.

Typical usage::

    import bteng_nav2.xml_integration as bt_nav2

    # 1. Register all Nav2 nodes with the global BTEng factory.
    bt_nav2.register_nodes()

    # 2. (Optional) Load a behaviour-tree XML that contains Nav2 nodes.
    root_node = bt_nav2.load_tree("my_bt.xml")
"""
from __future__ import annotations

from pathlib import Path

from bteng.blackboard.blackboard import Blackboard
from bteng.core.node import TreeNode
from bteng.factory.factory import NodeFactory
from bteng.xml_parser.parser import XMLTreeParser

from bteng_nav2.actions.compute_path import ComputePathThroughPosesNode, ComputePathToPoseNode
from bteng_nav2.actions.docking import DockRobotNode, UndockRobotNode
from bteng_nav2.actions.follow_path import FollowPathNode
from bteng_nav2.actions.navigate_through_poses import NavigateThroughPosesNode
from bteng_nav2.actions.navigate_to_pose import NavigateToPoseNode
from bteng_nav2.actions.recovery import (
    AssistedTeleopNode,
    BackUpNode,
    DriveOnHeadingNode,
    SpinNode,
    WaitNode,
)
from bteng_nav2.actions.smooth_path import SmoothPathNode
from bteng_nav2.conditions.nav_status import IsGoalUpdatedCondition, IsNavigationActiveCondition
from bteng_nav2.conditions.obstacle import IsObstacleNearCondition, IsStuckCondition
from bteng_nav2.conditions.pose import IsLocalizationActiveCondition, IsRobotNearGoalCondition
from bteng_nav2.conditions.robot_state import IsBatteryLowCondition, IsEmergencyStopCondition
from bteng_nav2.conditions.tf_condition import IsRobotAtFrameCondition
from bteng_nav2.publishers.goal import PublishGoalPoseNode
from bteng_nav2.publishers.localization import SetInitialPoseNode
from bteng_nav2.services.costmap import (
    ClearGlobalCostmapAroundRobotNode,
    ClearGlobalCostmapNode,
    ClearLocalCostmapAroundRobotNode,
    ClearLocalCostmapNode,
    GetGlobalCostmapNode,
    GetLocalCostmapNode,
)
from bteng_nav2.services.lifecycle import ManageLifecycleNodesNode
from bteng_nav2.services.map import LoadMapNode, SaveMapNode
from bteng_nav2.services.params import GetCostmapParameterNode, SetCostmapParameterNode
from bteng_nav2.services.path import IsPathValidNode
from bteng_nav2.services.slam import SaveSlamMapNode, SerializePoseGraphNode

__all__ = [
    "BTENG_NODES",
    "load_tree",
    "port_model",
    "register_nodes",
    "registered_tags",
]

# The classes are listed explicitly (imported, not looked up by string) so a
# rename or a moved module fails loudly at import time instead of silently
# dropping a node from the registry.
_NODE_CLASSES: tuple[type[TreeNode], ...] = (
    # Actions
    NavigateToPoseNode,
    NavigateThroughPosesNode,
    FollowPathNode,
    ComputePathToPoseNode,
    ComputePathThroughPosesNode,
    SmoothPathNode,
    # Recovery actions
    SpinNode,
    BackUpNode,
    DriveOnHeadingNode,
    WaitNode,
    AssistedTeleopNode,
    # Services
    ClearGlobalCostmapNode,
    ClearLocalCostmapNode,
    ClearGlobalCostmapAroundRobotNode,
    ClearLocalCostmapAroundRobotNode,
    GetGlobalCostmapNode,
    GetLocalCostmapNode,
    IsPathValidNode,
    LoadMapNode,
    SaveMapNode,
    ManageLifecycleNodesNode,
    SaveSlamMapNode,
    SerializePoseGraphNode,
    SetCostmapParameterNode,
    GetCostmapParameterNode,
    # Conditions
    IsRobotNearGoalCondition,
    IsLocalizationActiveCondition,
    IsObstacleNearCondition,
    IsStuckCondition,
    IsGoalUpdatedCondition,
    IsNavigationActiveCondition,
    IsBatteryLowCondition,
    IsEmergencyStopCondition,
    IsRobotAtFrameCondition,
    # Docking actions
    DockRobotNode,
    UndockRobotNode,
    # Publishers
    PublishGoalPoseNode,
    SetInitialPoseNode,
)

# Suffixes stripped to obtain the short (primary) XML tag of a class.
_TAG_SUFFIXES = ("Node", "Condition")


def _tags_for(node_class: type[TreeNode]) -> list[str]:
    """Return the XML tags for *node_class*, short (primary) form first.

    The short form is the class name without a trailing ``Node``/``Condition``
    suffix; the class name itself is kept as an alias.  A class whose name is
    exactly a suffix (or has none) contributes only its class name.
    """
    name = node_class.__name__
    for suffix in _TAG_SUFFIXES:
        if name.endswith(suffix) and len(name) > len(suffix):
            return [name[: -len(suffix)], name]
    return [name]


def _build_bteng_nodes(
    node_classes: tuple[type[TreeNode], ...],
) -> list[tuple[str, type[TreeNode]]]:
    """Expand *node_classes* into ``(tag, class)`` pairs, rejecting duplicates.

    A duplicate tag would make one class silently unreachable from XML, so it
    is a hard error here rather than a surprise at parse time.
    """
    pairs: list[tuple[str, type[TreeNode]]] = []
    owner: dict[str, type[TreeNode]] = {}
    for node_class in node_classes:
        for tag in _tags_for(node_class):
            if tag in owner:
                raise ValueError(
                    f"Duplicate XML tag {tag!r}: claimed by both "
                    f"{owner[tag].__name__} and {node_class.__name__}"
                )
            owner[tag] = node_class
            pairs.append((tag, node_class))
    return pairs


#: ``(xml_tag, node_class)`` pairs registered by :func:`register_nodes`.
#: Also consumed directly by ``NodeFactory.load_module("bteng_nav2")``.
BTENG_NODES: list[tuple[str, type[TreeNode]]] = _build_bteng_nodes(_NODE_CLASSES)


def register_nodes(factory: NodeFactory | None = None) -> None:
    """Register all bteng_nav2 nodes with BTEng's NodeFactory.

    Each entry in :data:`BTENG_NODES` — a list of ``(name: str, cls: type)``
    pairs — is registered with the factory under the given name.  Calling this
    function multiple times is safe; re-registering an already-registered name
    simply overwrites the entry with the same class.

    Parameters
    ----------
    factory:
        The ``NodeFactory`` instance to use.  Defaults to the global singleton
        (``NodeFactory.get_instance()``), which is what ``XMLTreeParser`` uses
        when no factory is supplied explicitly.
    """
    f = factory or NodeFactory.get_instance()
    for name, cls in BTENG_NODES:
        f.register(cls, name)

    # bteng ships SetBlackboard/CheckBlackboard but its factory does not
    # auto-register them; do it here so a tree using <SetBlackboard> builds
    # without the consumer registering anything by hand.
    try:
        from bteng.nodes.leaf.builtins import CheckBlackboard, SetBlackboard
    except ImportError:  # pragma: no cover - a bteng that lacks these builtins
        pass
    else:
        f.register(SetBlackboard, "SetBlackboard")
        f.register(CheckBlackboard, "CheckBlackboard")


def port_model() -> dict[str, dict[str, str]]:
    """Return ``{xml_tag: {port_name: "input_port" | "output_port"}}`` for our nodes.

    ``XMLTreeParser`` decides whether ``attr="{key}"`` becomes an input or an
    output port from its port model, which is otherwise filled in *only* from a
    ``<TreeNodesModel>`` block in the XML file and defaults to ``input_port``.
    Without this table ``<ComputePathToPose path="{path}"/>`` would bind ``path``
    as an input and the computed path would never reach the blackboard — silently.
    Since every node class already declares its ports in ``provided_ports()``,
    derive the directions from there instead of asking tree authors to repeat
    them in XML.

    A bidirectional port is reported as ``output_port``: the parser's model is
    binary and a lost write is the harder failure to notice.  (No bteng_nav2 node
    currently declares one.)
    """
    model: dict[str, dict[str, str]] = {}
    for tag, cls in BTENG_NODES:
        provided = getattr(cls, "provided_ports", None)
        if provided is None:
            continue
        try:
            ports = provided()
        except Exception:  # pragma: no cover - a node with a broken port decl
            continue
        model[tag] = {
            port.name: "output_port" if port.is_output() else "input_port" for port in ports
        }
    return model


def load_tree(
    path: str | Path,
    tree_id: str | None = None,
    blackboard: Blackboard | None = None,
    factory: NodeFactory | None = None,
) -> TreeNode:
    """Load a BTEng XML behaviour tree with all bteng_nav2 nodes pre-registered.

    This is a convenience wrapper that:

    1. Calls :func:`register_nodes` so Nav2 node types are available.
    2. Seeds the parser's port model from :func:`port_model` so that
       ``attr="{key}"`` on one of our output ports becomes an *output* port.
    3. Parses the XML file with :class:`bteng.xml_parser.parser.XMLTreeParser`.

    Step 2 writes to the parser's private ``_port_model`` because the class
    offers no public seeding hook; the attribute is a plain
    ``{tag: {port: direction}}`` dict and is stable across bteng 0.2.x/0.3.x.  A
    ``<TreeNodesModel>`` block inside the file still wins, since the parser loads
    it during :meth:`parse_file` and replaces the entry for the tags it declares.

    Parameters
    ----------
    path:
        Path to the ``.xml`` file in BTEng XML format.
    tree_id:
        If the file contains multiple ``<Tree ID="...">`` elements, specify
        which one to build.  Defaults to the first (or the one marked by
        ``main_tree_to_execute``).
    blackboard:
        An existing ``Blackboard`` instance to use.  A fresh one is created
        when omitted.
    factory:
        The ``NodeFactory`` instance to register nodes into and to pass to the
        parser.  Defaults to the global singleton.

    Returns
    -------
    TreeNode
        The root node of the parsed behaviour tree.
    """
    f = factory or NodeFactory.get_instance()
    register_nodes(f)
    parser = XMLTreeParser(factory=f)
    parser._port_model.update(port_model())
    root = parser.parse_file(str(path), tree_id=tree_id, blackboard=blackboard)
    coerce_literal_params(root)
    return root


#: How a literal XML attribute is turned into the type its port declares.
_LITERAL_COERCIONS = {
    bool: lambda text: {"true": True, "false": False, "1": True, "0": False}[
        text.strip().lower()
    ],
    int: lambda text: int(text.strip()),
    float: lambda text: float(text.strip()),
}


def coerce_literal_params(root: TreeNode) -> None:
    """Give every literal XML attribute the type its port declares.

    XML has no types: ``<Wait wait_duration="3"/>`` hands the node the *string*
    ``"3"``, and ``<ManageLifecycleNodes command="0"/>`` the string ``"0"``.  A
    blackboard-bound value (``"{key}"``) arrives correctly typed from the params
    file, which is why this went unnoticed -- every shipped tree used one, and
    every offline test seeded the blackboard.

    Put a string in a ROS message's numeric field and rclpy does not raise: the
    C extension **aborts the process**::

        nav2_msgs__srv__manage_lifecycle_nodes__request__convert_from_py:
        Assertion `PyLong_Check(field)' failed.

    A core dump, not a FAILURE -- no summary, no exit code, nothing for a
    supervisor to act on.  This is the first live run of ``05_full_autonomy``.

    Coercing here rather than in each node covers all 38 at once and keeps the
    node bodies free of parsing.  The declared default supplies the type; a port
    with a string default (or none) is left alone, and a value that will not
    convert raises now -- during ``--dry-run``, naming the node and port --
    rather than aborting later on the robot.
    """
    for node in _walk(root):
        params = getattr(getattr(node, "config", None), "params", None)
        if not params:
            continue
        for port in type(node).provided_ports():
            value = params.get(port.name)
            if not isinstance(value, str):
                continue
            # The declared type first, then the default's type. A required
            # numeric port has no default to infer from -- dist_to_travel and
            # speed on DriveOnHeading are two -- and inferring from the literal
            # instead would turn a dock_id of "0123" into an int.
            declared = getattr(port, "type_hint", None)
            convert = _LITERAL_COERCIONS.get(declared) or _LITERAL_COERCIONS.get(
                type(port.default)
            )
            if convert is None:  # a string or message port: nothing to convert
                continue
            try:
                params[port.name] = convert(value)
            except (KeyError, ValueError) as exc:
                raise ValueError(
                    f"{type(node).__name__} '{node.name}': port '{port.name}' expects "
                    f"{type(port.default).__name__}, got {value!r}"
                ) from exc


def _walk(node: TreeNode):
    yield node
    for child in getattr(node, "get_children", list)() or []:
        yield from _walk(child)


def registered_tags() -> list[str]:
    """Return the sorted XML tags this package contributes to the factory.

    Useful for tooling and for asserting in tests that a tag is ours rather
    than a BTEng built-in.
    """
    return sorted(tag for tag, _cls in BTENG_NODES)
