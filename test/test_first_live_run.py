"""The defects the first run against a real ROS graph produced.

Every one of these passed the whole offline suite and `--dry-run` and then died
on a live graph, so each test here exists to make sure the offline suite can
never say yes to them again.

The run: ROS 2 Jazzy in a container, real rclpy, real DDS, stub servers on the
Nav2 endpoints the trees address (a simulator was not needed -- what was missing
was a *wire*).  Three bugs, and the same root cause behind them: a
`MagicMock` answers to every attribute name, so nothing offline can tell the
difference between the object a node expects and the one ROS actually hands it.
"""
from __future__ import annotations

from types import SimpleNamespace

import pytest
from bteng import Blackboard, NodeConfig

from bteng_nav2.actions._result import action_result_of
from bteng_nav2.actions.compute_path import ComputePathThroughPosesNode, ComputePathToPoseNode
from bteng_nav2.actions.docking import DockRobotNode
from bteng_nav2.actions.navigate_to_pose import NavigateToPoseNode
from bteng_nav2.actions.smooth_path import SmoothPathNode

# ── Bug 1: an optional port declared the way a required one is ──────────────────


def test_start_is_optional_because_the_docs_and_make_goal_say_so():
    """`InputPort("start", default=None)` made `start` *required*: BTEng reads a
    None default as "no default". Every tree that planned from the robot's
    current pose -- the documented behaviour, and what make_goal() implements --
    was rejected at executor construction on the robot, while --dry-run passed."""
    for cls in (ComputePathToPoseNode, ComputePathThroughPosesNode):
        start = next(p for p in cls.provided_ports() if p.name == "start")
        assert start.default is not None, (
            f"{cls.__name__}.start has a None default, which BTEng treats as required"
        )


def test_a_tree_that_omits_start_validates():
    """The end-to-end version of the test above, through the real validator."""
    from bteng.core.validation import validate_tree

    bb = Blackboard()
    bb.set("goal", object())
    node = ComputePathToPoseNode(
        "plan",
        config=NodeConfig(
            blackboard=bb, input_ports={"goal": "goal"}, output_ports={"path": "path"}
        ),
    )
    assert validate_tree(node) == [], "a tree omitting the optional start port must validate"


def test_either_spelling_of_the_target_still_validates_alone():
    """goal_pose and goal are two spellings of one port, so both need a default
    -- which means validation can no longer require one of them. on_start()
    carries that check now, and this is the pair of trees it has to accept."""
    from bteng.core.validation import validate_tree

    for bound in ("goal_pose", "goal"):
        bb = Blackboard()
        bb.set(bound, object())
        node = ComputePathToPoseNode(
            "plan", config=NodeConfig(blackboard=bb, input_ports={bound: bound})
        )
        assert validate_tree(node) == [], f"binding only {bound} must validate"


def test_a_node_with_no_target_at_all_fails_with_both_names(monkeypatch):
    """...and a tree that binds neither must still fail, since validation cannot
    catch it any more -- naming both spellings, not sending an empty goal."""
    from bteng import NodeStatus

    node = ComputePathToPoseNode("plan", config=NodeConfig(blackboard=Blackboard()))
    node.set_ros_node(SimpleNamespace(get_logger=lambda: SimpleNamespace(error=lambda *a: None)))

    assert node.on_start() == NodeStatus.FAILURE
    assert "goal_pose" in node.feedback_message and "goal" in node.feedback_message


# ── Bug 2: action_result is the GetResult response, not the Result ──────────────


def test_action_result_of_unwraps_the_getresult_response():
    """`'ComputePathToPose_GetResult_Response' object has no attribute 'path'`.

    bteng-ros2 documents action_result as "Result message from the last
    completed action" but stores what get_result_async() resolved to -- the
    response wrapper. Its own code proves it: it reads action_result.status
    against GoalStatus, and a Result message has no status.
    """
    payload = SimpleNamespace(path="the path")
    node = SimpleNamespace(action_result=SimpleNamespace(status=4, result=payload))
    assert action_result_of(node) is payload


def test_action_result_of_accepts_a_bare_result_too():
    """Unwrapping rather than pinning the shape: if the boundary ever changes
    to hand over the Result directly, the nodes keep working."""
    payload = SimpleNamespace(path="the path")
    assert action_result_of(SimpleNamespace(action_result=payload)) is payload
    assert action_result_of(SimpleNamespace(action_result=None)) is None


@pytest.mark.parametrize(
    "cls, field, port",
    [
        (ComputePathToPoseNode, "path", "path"),
        (ComputePathThroughPosesNode, "path", "path"),
        (SmoothPathNode, "path", "smoothed_path"),
        (DockRobotNode, "success", "success"),
    ],
)
def test_every_payload_reading_node_reads_through_the_wrapper(cls, field, port):
    """The four nodes that read a field off the result. Each raised
    AttributeError on its first live tick; none of their tests noticed, because
    the fixture was a MagicMock, which has every attribute there is."""
    bb = Blackboard()
    node = cls("n", config=NodeConfig(blackboard=bb, output_ports={port: "out"}))
    node._RosActionClientMixin__action_result = SimpleNamespace(
        status=4, result=SimpleNamespace(**{field: "payload"})
    )
    node.on_success()
    assert bb.get("out") == "payload"


def test_the_result_port_carries_the_result_not_the_wrapper():
    """NavigateToPose's `result` port was accidentally right -- `.result` on the
    wrapper is the Result message. Pinned so the fix did not silently redefine
    what that port carries."""
    bb = Blackboard()
    node = NavigateToPoseNode("n", config=NodeConfig(blackboard=bb, output_ports={"result": "r"}))
    payload = SimpleNamespace(error_code=0)
    node._RosActionClientMixin__action_result = SimpleNamespace(status=4, result=payload)
    node.on_success()
    assert bb.get("r") is payload


# ── Bug 3: the gate that could not see either of them ───────────────────────────


def test_dry_run_validates_ports(tmp_path, capsys):
    """--dry-run built the tree and never validated it, so the one command whose
    job is catching a broken tree before the robot could not catch an unbound
    required port. That is how bug 1 reached a live run."""
    from bteng_nav2 import runtime

    tree = tmp_path / "unbound.xml"
    tree.write_text(
        '<?xml version="1.0" encoding="UTF-8"?>\n'
        '<BTEng format_version="1.0" main_tree_to_execute="main">\n'
        '  <Tree ID="main">\n'
        '    <ComputePathThroughPoses name="plan" path="{path}" />\n'
        "  </Tree>\n"
        "</BTEng>\n",
        encoding="utf-8",
    )
    params = tmp_path / "params.yaml"
    params.write_text("ros: {}\nblackboard: {}\n")

    exit_code = runtime.run_trees([str(tree)], params, dry_run=True, pretty=False)
    assert exit_code == 1, "a tree with an unbound required port must fail the CI gate"


def test_dry_run_still_passes_every_shipped_example():
    """The other half: validation must not reject the trees this package ships."""
    from bteng_nav2 import runtime

    assert runtime.run_trees(
        ["examples/trees/"], "examples/config/nav2_params.yaml", dry_run=True, pretty=False
    ) == 0


# ── Bug 4: XML has no types, and rclpy does not raise -- it aborts ──────────────


def test_a_literal_number_in_xml_reaches_the_node_as_a_number(tmp_path):
    """`<ManageLifecycleNodes command="0"/>` handed the node the string "0".

    Put a str in a ROS message's numeric field and the C extension does not
    raise, it **aborts the process**:

        nav2_msgs__srv__manage_lifecycle_nodes__request__convert_from_py:
        Assertion `PyLong_Check(field)' failed.

    A core dump, not a FAILURE: no summary, no exit code, nothing a supervisor
    can act on. It hid because every shipped tree and every offline test bound
    these ports to the blackboard, where the params file had already typed them.
    """
    from bteng_nav2.xml_integration import load_tree

    tree = tmp_path / "literals.xml"
    tree.write_text(
        '<?xml version="1.0" encoding="UTF-8"?>\n'
        '<BTEng format_version="1.0" main_tree_to_execute="main">\n'
        '  <Tree ID="main">\n'
        '    <Sequence name="s">\n'
        '      <ManageLifecycleNodes name="startup" command="0" />\n'
        '      <BackUp name="back" backup_dist="0.30" time_allowance="7" />\n'
        '      <Wait name="pause" wait_duration="3" />\n'
        "    </Sequence>\n"
        "  </Tree>\n"
        "</BTEng>\n",
        encoding="utf-8",
    )
    root = load_tree(tree, blackboard=Blackboard())
    startup, back, pause = root.get_children()

    assert startup.config.params["command"] == 0
    assert isinstance(startup.config.params["command"], int)
    assert back.config.params["backup_dist"] == pytest.approx(0.30)
    assert isinstance(back.config.params["backup_dist"], float)
    assert pause.config.params["wait_duration"] == 3
    assert isinstance(pause.config.params["wait_duration"], int)


def test_a_string_port_is_left_alone(tmp_path):
    """Only ports whose declared default is numeric get coerced -- a planner id
    or an endpoint is a string and must stay one."""
    from bteng_nav2.xml_integration import load_tree

    tree = tmp_path / "strings.xml"
    tree.write_text(
        '<?xml version="1.0" encoding="UTF-8"?>\n'
        '<BTEng format_version="1.0" main_tree_to_execute="main">\n'
        '  <Tree ID="main">\n'
        '    <ComputePathToPose name="plan" goal_pose="{goal}" planner_id="GridBased"\n'
        '                       action_name="/planner_b/compute_path_to_pose" />\n'
        "  </Tree>\n"
        "</BTEng>\n",
        encoding="utf-8",
    )
    bb = Blackboard()
    bb.set("goal", object())
    node = load_tree(tree, blackboard=bb)
    assert node.config.params["planner_id"] == "GridBased"
    assert node.config.params["action_name"] == "/planner_b/compute_path_to_pose"


def test_a_number_that_will_not_parse_fails_at_build_naming_the_port(tmp_path):
    """A typo'd literal must fail here -- during --dry-run, in CI -- rather than
    reaching the robot and aborting the process mid-run."""
    from bteng_nav2.xml_integration import load_tree

    tree = tmp_path / "typo.xml"
    tree.write_text(
        '<?xml version="1.0" encoding="UTF-8"?>\n'
        '<BTEng format_version="1.0" main_tree_to_execute="main">\n'
        '  <Tree ID="main">\n'
        '    <BackUp name="back" backup_dist="0.3m" />\n'
        "  </Tree>\n"
        "</BTEng>\n",
        encoding="utf-8",
    )
    with pytest.raises(ValueError) as err:
        load_tree(tree, blackboard=Blackboard())
    assert "backup_dist" in str(err.value) and "0.3m" in str(err.value)


# ── Bug 5: a required numeric port has no default to infer a type from ──────────


def test_required_numeric_ports_declare_their_type(tmp_path):
    """Coercing from the declared default cannot help a port that has none.

    `<DriveOnHeading dist_to_travel="0.5" speed="0.1"/>` still handed the node
    strings, and geometry_msgs aborted the process on the Point field:

        geometry_msgs__msg__point__convert_from_py:
        Assertion `PyFloat_Check(field)' failed.

    Inferring the type from the literal instead would be worse -- a dock_id of
    "0123" would silently become an int -- so these ports carry a type_hint.
    """
    from bteng_nav2.actions.recovery import DriveOnHeadingNode, SpinNode
    from bteng_nav2.services.params import SetCostmapParameterNode
    from bteng_nav2.xml_integration import load_tree

    for cls, names in (
        (DriveOnHeadingNode, ("dist_to_travel", "speed")),
        (SpinNode, ("target_yaw",)),
        (SetCostmapParameterNode, ("param_value",)),
    ):
        ports = {p.name: p for p in cls.provided_ports()}
        for name in names:
            port = ports[name]
            assert port.default is not None or port.type_hint is not None, (
                f"{cls.__name__}.{name} is numeric, required, and untyped: a literal "
                f"in XML would reach a ROS message as a string and abort the process"
            )

    tree = tmp_path / "drive.xml"
    tree.write_text(
        '<?xml version="1.0" encoding="UTF-8"?>\n'
        '<BTEng format_version="1.0" main_tree_to_execute="main">\n'
        '  <Tree ID="main">\n'
        '    <DriveOnHeading name="d" dist_to_travel="0.5" speed="0.1" />\n'
        "  </Tree>\n"
        "</BTEng>\n",
        encoding="utf-8",
    )
    node = load_tree(tree, blackboard=Blackboard())
    assert node.config.params["dist_to_travel"] == pytest.approx(0.5)
    assert isinstance(node.config.params["dist_to_travel"], float)
    assert isinstance(node.config.params["speed"], float)


# ── Bug 6: the docking messages moved into Nav2 ─────────────────────────────────


def test_docking_finds_its_messages_in_nav2_msgs(monkeypatch):
    """`No module named 'opennav_docking_msgs'` on a stock Jazzy install.

    Docking began in the standalone opennav_docking stack and was merged into
    Nav2, so the package depends on the distribution. These nodes imported the
    standalone name only, and every docking tree died on its first tick.
    """
    import sys
    import types

    from bteng_nav2.actions.docking import _docking_action

    nav2_action = sys.modules["nav2_msgs.action"]
    sentinel = type("DockRobot", (), {})
    monkeypatch.setattr(nav2_action, "DockRobot", sentinel, raising=False)
    assert _docking_action("DockRobot") is sentinel, "nav2_msgs must be preferred"

    # ...and an install that still ships the standalone package keeps working.
    monkeypatch.delattr(nav2_action, "DockRobot")
    standalone = types.ModuleType("opennav_docking_msgs.action")
    standalone.DockRobot = type("DockRobot", (), {})
    monkeypatch.setitem(sys.modules, "opennav_docking_msgs.action", standalone)
    assert _docking_action("DockRobot") is standalone.DockRobot


def test_docking_says_it_means_the_dock_id_it_was_given():
    """Nav2's DockRobot goal carries both a dock id and a dock pose, and reads
    the pose unless use_dock_id is set. A goal naming a dock by id and leaving
    the flag at its default False sends the server to an empty pose."""
    from bteng_nav2.actions.docking import DockRobotNode

    class _Goal:
        """Nav2's shape: the flag defaults to False, as it does on the wire."""

        def __init__(self):
            self.dock_id = ""
            self.dock_type = ""
            self.dock_pose = None
            self.use_dock_id = False

    monkeypatch = pytest.MonkeyPatch()
    monkeypatch.setattr(
        "bteng_nav2.actions.docking._docking_action",
        lambda name: type("DockRobot", (), {"Goal": _Goal}),
    )
    try:
        bb = Blackboard()
        bb.set("dock_id", "charger_01")
        node = DockRobotNode(
            "d", config=NodeConfig(blackboard=bb, input_ports={"dock_id": "dock_id"})
        )
        goal = node.make_goal()
    finally:
        monkeypatch.undo()

    assert goal.dock_id == "charger_01"
    assert goal.use_dock_id is True, "the server reads dock_pose unless this is set"


# ── Bug 7: a field that looks like a string but is a message ────────────────────


def test_slam_save_map_wraps_the_filename_in_a_message():
    """slam_toolbox declares SaveMap.Request.name as std_msgs/String -- a
    *message* -- while SerializePoseGraph.Request.filename, in the same package,
    is a plain string. Assigning a str to the first aborts the process:

        std_msgs__msg__string__convert_from_py:
        Assertion `strncmp("std_msgs.msg._string.String", ...) == 0' failed.

    The test that covered this asserted `req.name == "my_map"` against a
    MagicMock, which is exactly the shape that crashes.
    """
    import sys
    import types

    from bteng_nav2.services.slam import SaveSlamMapNode, SerializePoseGraphNode

    # Self-contained stubs shaped like the real service definitions, so this
    # test states the contract rather than inheriting whatever another module
    # installed. test_slam.py's stub arrives only when that file is imported.
    monkeypatch = pytest.MonkeyPatch()
    srv = types.ModuleType("slam_toolbox.srv")

    class _SaveMapRequest:
        def __init__(self):
            self.name = None

    class _SerializeRequest:
        def __init__(self):
            self.filename = ""

    srv.SaveMap = type("SaveMap", (), {"Request": _SaveMapRequest})
    srv.SerializePoseGraph = type("SerializePoseGraph", (), {"Request": _SerializeRequest})
    monkeypatch.setitem(sys.modules, "slam_toolbox", types.ModuleType("slam_toolbox"))
    monkeypatch.setitem(sys.modules, "slam_toolbox.srv", srv)

    bb = Blackboard()
    bb.set("filename", "/tmp/map")

    saver = SaveSlamMapNode(
        "save", config=NodeConfig(blackboard=bb, input_ports={"filename": "filename"})
    )
    request = saver.make_request()
    assert not isinstance(request.name, str), "name is std_msgs/String, not str"
    assert request.name.data == "/tmp/map"

    serializer = SerializePoseGraphNode(
        "graph", config=NodeConfig(blackboard=bb, input_ports={"filename": "filename"})
    )
    assert serializer.make_request().filename == "/tmp/map"
    monkeypatch.undo()
