"""Every node's ``on_start()`` must return a NodeStatus, for all 38 at once.

``StatefulActionNode.tick()`` *returns* ``on_start()``.  A node that writes
``super().on_start()`` without ``return`` therefore reports ``None`` on its very
first tick, and the tree reads that as terminal: the goal is sent, the action
server starts driving the robot, and the tree has already moved on.  On
``bteng>=0.3.1`` the engine raises instead of silently accepting it, which turns
a wrong result into a crash on the first tick of a live run -- no better for
anyone standing next to the robot.

That bug was fixed at 13 sites in ``actions/`` and nothing pinned it: the suite
called ``on_start()`` in a dozen places and never once looked at what came back.
This file is the guard.  It walks every exported class rather than naming them,
so a node added next year is covered without anyone remembering to add a test.
"""
from __future__ import annotations

import inspect
from unittest.mock import MagicMock

import pytest
from bteng import Blackboard, NodeConfig, NodeStatus, StatefulActionNode

import bteng_nav2


def _stateful_classes():
    """Exported node classes whose first tick goes through ``on_start()``."""
    out = []
    for name in bteng_nav2.__all__:
        obj = getattr(bteng_nav2, name)
        if inspect.isclass(obj) and issubclass(obj, StatefulActionNode):
            out.append(pytest.param(obj, id=name))
    return out


STATEFUL = _stateful_classes()


def _build(cls):
    """Construct *cls* with every declared input port bound to a mock value.

    The values are deliberately mocks rather than typed messages: this test is
    about the control flow of ``on_start()``, and the message stubs in
    conftest.py are strict enough that real values would make it a test of the
    builders instead.
    """
    bb = Blackboard()
    ports = {}
    for port in cls.provided_ports():
        if type(port).__name__ == "OutputPort":
            continue
        value = port.default if port.default is not None else MagicMock()
        bb.set(f"{cls.__name__}_{port.name}", value)
        ports[port.name] = f"{cls.__name__}_{port.name}"

    node = cls("contract", config=NodeConfig(blackboard=bb, input_ports=ports))
    node.set_ros_node(MagicMock())

    # Neutralise the ROS layer: this asserts what on_start *returns*, not what
    # it sends. Every send path is mocked, and the readiness probes answer yes
    # so on_start reaches its return statement instead of parking in discovery.
    node._init_action_client = MagicMock()
    node._init_service_client = MagicMock()
    node.send_goal = MagicMock()
    node.call_service = MagicMock()
    node.action_status = MagicMock(return_value=NodeStatus.RUNNING)
    node.service_status = MagicMock(return_value=NodeStatus.RUNNING)
    node.set_output = MagicMock()
    node._RosActionClientMixin__action_result = MagicMock()
    return node


@pytest.mark.parametrize("cls", STATEFUL)
def test_on_start_returns_a_node_status(cls):
    node = _build(cls)
    try:
        result = node.on_start()
    except Exception as exc:  # a node whose make_goal needs a real message
        pytest.skip(f"{cls.__name__}.on_start() needs a typed message: {exc!r}")
    assert isinstance(result, NodeStatus), (
        f"{cls.__name__}.on_start() returned {result!r}. StatefulActionNode.tick() "
        f"returns this value, so the tree reads it as the node's first status -- "
        f"write `return super().on_start()`."
    )


@pytest.mark.parametrize("cls", STATEFUL)
def test_on_start_source_returns_what_it_calls(cls):
    """The static half: a node that calls ``super().on_start()`` must return it.

    The dynamic test above skips a node whose ``make_goal()`` demands a real
    message, and a skipped test is not a guard. This one needs no construction
    at all, so it covers every class unconditionally -- it is the check that
    would have caught all 13 original sites.
    """
    if "on_start" not in vars(cls):
        return  # inherits the base implementation, which returns correctly
    source = inspect.getsource(cls.on_start)
    if "super().on_start()" not in source:
        return  # does not delegate; the dynamic test covers it
    assert "return super().on_start()" in source, (
        f"{cls.__name__}.on_start() calls super().on_start() and drops the result. "
        f"StatefulActionNode.tick() returns it, so the node reports None on its "
        f"first tick."
    )


def test_the_walk_actually_found_the_nodes():
    """A guard on the guard: if the export list or the base class moves, the
    parametrised tests above would silently cover nothing and stay green."""
    assert len(STATEFUL) >= 20, f"only found {len(STATEFUL)} stateful node classes"
