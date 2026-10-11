"""Docking action nodes for Nav2."""

from __future__ import annotations

from bteng import InputPort, OutputPort
from bteng_ros2 import RosActionNode

from bteng_nav2.actions._result import action_result_of


def _docking_action(name: str):
    """The docking action type, from wherever this installation keeps it.

    Docking started life in the standalone ``opennav_docking`` stack and was
    merged into Nav2, so the message package differs by distribution: Jazzy and
    newer ship ``nav2_msgs.action.DockRobot``, while an install that still uses
    the standalone stack has ``opennav_docking_msgs``.  These nodes imported the
    standalone package only, so on a stock Jazzy machine every docking tree died
    on its first tick with ``No module named 'opennav_docking_msgs'`` -- found on
    the first live run, since a stubbed test never reaches the import.
    """
    import importlib

    errors = []
    for module in ("nav2_msgs.action", "opennav_docking_msgs.action"):
        try:
            return getattr(importlib.import_module(module), name)
        except (ImportError, AttributeError) as exc:  # noqa: PERF203 - two tries, once
            errors.append(f"{module}: {exc}")
    raise ImportError(
        f"{name} not found. Install Nav2 (nav2_msgs) or the standalone docking "
        f"stack (opennav_docking_msgs). Tried -- " + "; ".join(errors)
    )


class DockRobotNode(RosActionNode):
    """Send a DockRobot goal to the Nav2 docking server."""

    action_name = "/dock_robot"

    @classmethod
    def provided_ports(cls):
        return [
            InputPort("dock_id"),
            InputPort("dock_type", default=""),
            OutputPort("success"),
        ]

    def on_start(self):
        self.action_type = _docking_action("DockRobot")
        return super().on_start()

    def make_goal(self):
        goal = _docking_action("DockRobot").Goal()
        goal.dock_id = self.get_input("dock_id")
        goal.dock_type = self.get_input("dock_type", "")
        # Without this the server reads dock_pose instead and ignores the id we
        # just set -- it defaults to False, and a goal that names a dock by id
        # means to use it.
        if hasattr(goal, "use_dock_id"):
            goal.use_dock_id = True
        return goal

    def on_success(self):
        self.set_output("success", action_result_of(self).success)


class UndockRobotNode(RosActionNode):
    """Send an UndockRobot goal to the Nav2 docking server."""

    action_name = "/undock_robot"

    @classmethod
    def provided_ports(cls):
        return [
            InputPort("dock_type", default=""),
        ]

    def on_start(self):
        self.action_type = _docking_action("UndockRobot")
        return super().on_start()

    def make_goal(self):
        goal = _docking_action("UndockRobot").Goal()
        goal.dock_type = self.get_input("dock_type", "")
        return goal
