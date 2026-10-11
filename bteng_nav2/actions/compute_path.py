"""ComputePathToPose and ComputePathThroughPoses action nodes for Nav2."""

from __future__ import annotations

from bteng import InputPort, NodeStatus, OutputPort
from bteng_ros2 import RosActionNode

from bteng_nav2.actions._result import action_result_of

#: What an optional port carries when the tree said nothing.
#:
#: It cannot be ``None``: BTEng reads a ``None`` default as "this port has no
#: default", which makes the port *required* -- so ``InputPort("start",
#: default=None)`` rejected at validation every tree that did not bind ``start``,
#: while this file's own ``make_goal()`` and the docs both describe it as
#: optional.  ``--dry-run`` never validated, so the shipped example that omitted
#: it passed CI and failed on the first real run.
_UNSET = ""


def _is_set(value) -> bool:
    """True when a tree actually supplied a value for an optional port."""
    return value is not None and value is not _UNSET and value != _UNSET


class ComputePathToPoseNode(RosActionNode):
    """Compute a path to a single goal pose via the Nav2 planner.

    Takes the target on ``goal_pose`` or on ``goal``.  NavigateToPose spells it
    ``goal_pose``, this node spelled it ``goal``, and a tree that wired the two
    together had to remember which was which; ``goal_pose`` is now the preferred
    name here too and ``goal`` keeps working.
    """

    action_name = "/compute_path_to_pose"

    @classmethod
    def provided_ports(cls):
        return [
            InputPort("goal_pose", "Target pose (preferred spelling)", default=_UNSET),
            InputPort("goal", "Target pose (original spelling; still supported)",
                      default=_UNSET),
            InputPort("start", "Start pose; omit to plan from the robot's pose",
                      default=_UNSET),
            InputPort("planner_id", default=""),
            OutputPort("path"),
        ]

    def _target(self):
        """goal_pose wins; goal is the original spelling.  None if neither."""
        target = self.get_input("goal_pose", _UNSET)
        if not _is_set(target):
            target = self.get_input("goal", _UNSET)
        return target if _is_set(target) else None

    def on_start(self):
        from nav2_msgs.action import ComputePathToPose

        # Exactly one of the two spellings is required, and validation can no
        # longer say so: a port with a default is optional by definition, and
        # both need one so that binding *either* spelling validates. So the
        # check moves here, where it can name both names -- rather than sending
        # a goal with an unset target and letting the planner reject it.
        if self._target() is None:
            self.set_feedback_message(
                "no target: bind goal_pose= (preferred) or goal= on this node"
            )
            self.ros_logger().error(f"{self.name}: no target pose on goal_pose= or goal=")
            return NodeStatus.FAILURE

        self.action_type = ComputePathToPose
        return super().on_start()

    def make_goal(self):
        from nav2_msgs.action import ComputePathToPose

        goal = ComputePathToPose.Goal()
        goal.goal = self._target()
        start = self.get_input("start", _UNSET)
        if _is_set(start):
            goal.start = start
            goal.use_start = True
        goal.planner_id = self.get_input("planner_id", "")
        return goal

    def on_success(self):
        self.set_output("path", action_result_of(self).path)


class ComputePathThroughPosesNode(RosActionNode):
    """Compute a path through multiple goal poses via the Nav2 planner."""

    action_name = "/compute_path_through_poses"

    @classmethod
    def provided_ports(cls):
        return [
            InputPort("goals"),  # no default: genuinely required
            InputPort("start", "Start pose; omit to plan from the robot's pose",
                      default=_UNSET),
            InputPort("planner_id", default=""),
            OutputPort("path"),
        ]

    def on_start(self):
        from nav2_msgs.action import ComputePathThroughPoses

        self.action_type = ComputePathThroughPoses
        return super().on_start()

    def make_goal(self):
        from nav2_msgs.action import ComputePathThroughPoses

        goal = ComputePathThroughPoses.Goal()
        goal.goals = self.get_input("goals")
        start = self.get_input("start", _UNSET)
        if _is_set(start):
            goal.start = start
            goal.use_start = True
        goal.planner_id = self.get_input("planner_id", "")
        return goal

    def on_success(self):
        self.set_output("path", action_result_of(self).path)
