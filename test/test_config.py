"""Tests for bteng_nav2.config — the YAML -> ROS message params loader.

ROS message classes are the stubs installed by test/conftest.py, so these tests
check field values, not real message types.
"""

from __future__ import annotations

import math

import pytest
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import (
    Point,
    PoseStamped,
    PoseWithCovarianceStamped,
    Quaternion,
)
from nav_msgs.msg import Path as PathMsg

from bteng_nav2.config import (
    BUILDERS,
    ConfigError,
    Nav2Config,
    RosConfig,
    build_value,
    load_config,
    parse_bb_presets,
)

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def write(tmp_path, text, name="params.yaml"):
    path = tmp_path / name
    path.write_text(text, encoding="utf-8")
    return path


def yaw_of(quat):
    """Recover yaw from a yaw-only quaternion."""
    return 2.0 * math.atan2(quat.z, quat.w)


# ---------------------------------------------------------------------------
# Builder registry
# ---------------------------------------------------------------------------


def test_builders_cover_documented_types():
    assert set(BUILDERS) == {
        "pose",
        "pose_cov",
        "poses",
        "path",
        "point",
        "quaternion",
        "duration",
    }


# ---------------------------------------------------------------------------
# pose
# ---------------------------------------------------------------------------


def test_pose_builds_pose_stamped():
    msg = build_value({"type": "pose", "frame_id": "map", "x": 2.0, "y": 1.5, "yaw": 0.0}, "g")
    assert isinstance(msg, PoseStamped)
    assert msg.header.frame_id == "map"
    assert (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z) == (2.0, 1.5, 0.0)
    assert msg.pose.orientation.w == pytest.approx(1.0)


def test_pose_defaults_frame_and_zeros():
    msg = build_value({"type": "pose"}, "g")
    assert msg.header.frame_id == "map"
    assert (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z) == (0.0, 0.0, 0.0)
    assert (msg.pose.orientation.x, msg.pose.orientation.y) == (0.0, 0.0)
    assert (msg.pose.orientation.z, msg.pose.orientation.w) == (0.0, 1.0)


def test_pose_stamp_left_at_zero():
    """No ROS clock at load time; nav2/tf2 read a zero stamp as 'now'."""
    msg = build_value({"type": "pose"}, "g")
    assert msg.header.stamp is None or getattr(msg.header.stamp, "sec", 0) == 0


def test_pose_z_and_int_coercion():
    msg = build_value({"type": "pose", "x": 1, "z": 3}, "g")
    assert msg.pose.position.x == 1.0
    assert isinstance(msg.pose.position.x, float)
    assert msg.pose.position.z == 3.0


@pytest.mark.parametrize("yaw", [0.0, math.pi / 2, math.pi])
def test_yaw_to_quaternion_math(yaw):
    msg = build_value({"type": "pose", "yaw": yaw}, "g")
    quat = msg.pose.orientation
    assert quat.x == 0.0
    assert quat.y == 0.0
    assert quat.z == pytest.approx(math.sin(yaw / 2.0))
    assert quat.w == pytest.approx(math.cos(yaw / 2.0))
    assert quat.z**2 + quat.w**2 == pytest.approx(1.0)


def test_yaw_half_pi_exact_values():
    msg = build_value({"type": "pose", "yaw": math.pi / 2}, "g")
    assert msg.pose.orientation.z == pytest.approx(0.7071067811865476)
    assert msg.pose.orientation.w == pytest.approx(0.7071067811865476)


def test_explicit_quaternion_path():
    msg = build_value(
        {"type": "pose", "qx": 0.0, "qy": 0.0, "qz": 0.7071067811865476, "qw": 0.7071067811865476},
        "g",
    )
    assert yaw_of(msg.pose.orientation) == pytest.approx(math.pi / 2)


def test_yaw_and_quaternion_conflict_rejected():
    with pytest.raises(ConfigError) as err:
        build_value({"type": "pose", "yaw": 1.0, "qz": 0.0, "qw": 1.0}, "goal_pose")
    message = str(err.value)
    assert "goal_pose" in message
    assert "yaw" in message
    assert "qz" in message


def test_unnormalized_quaternion_rejected():
    with pytest.raises(ConfigError) as err:
        build_value({"type": "pose", "qz": 0.7}, "goal_pose")
    assert "goal_pose" in str(err.value)
    assert "normalized" in str(err.value)


def test_non_numeric_coordinate_rejected():
    with pytest.raises(ConfigError) as err:
        build_value({"type": "pose", "x": "over there"}, "goal_pose")
    assert "goal_pose" in str(err.value)
    assert "'x'" in str(err.value)


def test_non_string_frame_id_rejected():
    with pytest.raises(ConfigError) as err:
        build_value({"type": "pose", "frame_id": 3}, "goal_pose")
    assert "goal_pose" in str(err.value)
    assert "frame_id" in str(err.value)


# ---------------------------------------------------------------------------
# pose_cov
# ---------------------------------------------------------------------------


def test_pose_cov_builds_message():
    msg = build_value(
        {"type": "pose_cov", "frame_id": "map", "x": 1.0, "y": 2.0, "yaw": math.pi},
        "initial_pose",
    )
    assert isinstance(msg, PoseWithCovarianceStamped)
    assert msg.header.frame_id == "map"
    assert (msg.pose.pose.position.x, msg.pose.pose.position.y) == (1.0, 2.0)
    assert msg.pose.pose.orientation.z == pytest.approx(1.0)
    assert msg.pose.pose.orientation.w == pytest.approx(0.0, abs=1e-12)


def test_pose_cov_accepts_36_covariance():
    cov = [0.0] * 36
    cov[0] = 0.25
    msg = build_value({"type": "pose_cov", "covariance": cov}, "initial_pose")
    assert len(msg.pose.covariance) == 36
    assert msg.pose.covariance[0] == 0.25
    assert all(isinstance(v, float) for v in msg.pose.covariance)


def test_pose_cov_bad_covariance_length_rejected():
    with pytest.raises(ConfigError) as err:
        build_value({"type": "pose_cov", "covariance": [0.0] * 12}, "initial_pose")
    message = str(err.value)
    assert "initial_pose" in message
    assert "36" in message


def test_pose_cov_non_numeric_covariance_entry_rejected():
    cov = [0.0] * 36
    cov[3] = "nope"
    with pytest.raises(ConfigError) as err:
        build_value({"type": "pose_cov", "covariance": cov}, "initial_pose")
    assert "initial_pose" in str(err.value)


def test_pose_cov_covariance_must_be_list():
    with pytest.raises(ConfigError) as err:
        build_value({"type": "pose_cov", "covariance": 0.25}, "initial_pose")
    assert "initial_pose" in str(err.value)


# ---------------------------------------------------------------------------
# poses
# ---------------------------------------------------------------------------


def test_poses_builds_list_of_pose_stamped():
    msgs = build_value(
        {"type": "poses", "frame_id": "map", "points": [[1.0, 2.0], [3.0, 4.0, 1.57]]},
        "waypoints",
    )
    assert isinstance(msgs, list)
    assert len(msgs) == 2
    assert all(isinstance(m, PoseStamped) for m in msgs)
    assert all(m.header.frame_id == "map" for m in msgs)
    assert (msgs[0].pose.position.x, msgs[0].pose.position.y) == (1.0, 2.0)
    assert msgs[0].pose.orientation.w == pytest.approx(1.0)
    assert (msgs[1].pose.position.x, msgs[1].pose.position.y) == (3.0, 4.0)
    assert yaw_of(msgs[1].pose.orientation) == pytest.approx(1.57)


def test_poses_accepts_full_pose_mapping_entry():
    msgs = build_value(
        {
            "type": "poses",
            "frame_id": "map",
            "points": [{"x": 1.0, "y": 2.0, "z": 0.5, "frame_id": "odom"}, [0.0, 0.0]],
        },
        "waypoints",
    )
    assert msgs[0].header.frame_id == "odom"
    assert msgs[0].pose.position.z == 0.5
    assert msgs[1].header.frame_id == "map"


def test_poses_empty_and_missing_points():
    assert build_value({"type": "poses", "points": []}, "w") == []
    assert build_value({"type": "poses"}, "w") == []


def test_poses_bad_entry_arity_rejected():
    with pytest.raises(ConfigError) as err:
        build_value({"type": "poses", "points": [[1.0, 2.0, 3.0, 4.0]]}, "waypoints")
    message = str(err.value)
    assert "waypoints" in message
    assert "points[0]" in message


def test_poses_scalar_entry_rejected():
    with pytest.raises(ConfigError) as err:
        build_value({"type": "poses", "points": [1.0]}, "waypoints")
    assert "waypoints" in str(err.value)


def test_poses_points_must_be_list():
    with pytest.raises(ConfigError) as err:
        build_value({"type": "poses", "points": {"x": 1.0}}, "waypoints")
    assert "waypoints" in str(err.value)
    assert "points" in str(err.value)


def test_poses_entry_with_wrong_type_tag_rejected():
    with pytest.raises(ConfigError) as err:
        build_value({"type": "poses", "points": [{"type": "point", "x": 1.0}]}, "waypoints")
    assert "waypoints" in str(err.value)


# ---------------------------------------------------------------------------
# path
# ---------------------------------------------------------------------------


def test_path_builds_nav_msgs_path():
    msg = build_value(
        {"type": "path", "frame_id": "map", "points": [[0.0, 0.0], [1.0, 0.0]]}, "route"
    )
    assert isinstance(msg, PathMsg)
    assert msg.header.frame_id == "map"
    assert len(msg.poses) == 2
    assert all(isinstance(p, PoseStamped) for p in msg.poses)
    assert msg.poses[1].pose.position.x == 1.0
    assert msg.poses[1].header.frame_id == "map"


def test_path_unknown_key_rejected():
    with pytest.raises(ConfigError) as err:
        build_value({"type": "path", "poses": [[0.0, 0.0]]}, "route")
    message = str(err.value)
    assert "route" in message
    assert "'poses'" in message
    assert "points" in message


# ---------------------------------------------------------------------------
# point / quaternion / duration
# ---------------------------------------------------------------------------


def test_point_builder():
    msg = build_value({"type": "point", "x": 1.0, "y": 2.0, "z": 3.0}, "p")
    assert isinstance(msg, Point)
    assert (msg.x, msg.y, msg.z) == (1.0, 2.0, 3.0)


def test_point_defaults():
    msg = build_value({"type": "point"}, "p")
    assert (msg.x, msg.y, msg.z) == (0.0, 0.0, 0.0)


def test_point_rejects_frame_id():
    with pytest.raises(ConfigError) as err:
        build_value({"type": "point", "frame_id": "map"}, "target")
    assert "target" in str(err.value)
    assert "frame_id" in str(err.value)


def test_quaternion_from_yaw():
    msg = build_value({"type": "quaternion", "yaw": math.pi / 2}, "q")
    assert isinstance(msg, Quaternion)
    assert msg.z == pytest.approx(math.sin(math.pi / 4))
    assert msg.w == pytest.approx(math.cos(math.pi / 4))


def test_quaternion_explicit():
    msg = build_value({"type": "quaternion", "qx": 0.0, "qy": 0.0, "qz": 1.0, "qw": 0.0}, "q")
    assert (msg.x, msg.y, msg.z, msg.w) == (0.0, 0.0, 1.0, 0.0)


def test_quaternion_conflict_rejected():
    with pytest.raises(ConfigError) as err:
        build_value({"type": "quaternion", "yaw": 0.5, "qw": 1.0}, "heading")
    assert "heading" in str(err.value)


def test_duration_from_sec_and_nanosec():
    msg = build_value({"type": "duration", "sec": 5, "nanosec": 250}, "timeout")
    assert isinstance(msg, Duration)
    assert (msg.sec, msg.nanosec) == (5, 250)


def test_duration_from_seconds_float():
    msg = build_value({"type": "duration", "seconds": 1.5}, "timeout")
    assert msg.sec == 1
    assert msg.nanosec == pytest.approx(500_000_000, abs=10)


def test_duration_defaults_to_zero():
    msg = build_value({"type": "duration"}, "timeout")
    assert (msg.sec, msg.nanosec) == (0, 0)


def test_duration_mixed_forms_rejected():
    with pytest.raises(ConfigError) as err:
        build_value({"type": "duration", "seconds": 1.0, "sec": 2}, "timeout")
    assert "timeout" in str(err.value)


def test_duration_non_integer_sec_rejected():
    with pytest.raises(ConfigError) as err:
        build_value({"type": "duration", "sec": 1.5}, "timeout")
    assert "timeout" in str(err.value)
    assert "'sec'" in str(err.value)


# ---------------------------------------------------------------------------
# build_value rules
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    "value",
    [0.15, 5, "dock_a", True, False, None, [1.0, 2.0], ["a", "b"], []],
)
def test_scalars_and_scalar_lists_pass_through(value):
    assert build_value(value, "k") is value


def test_nested_specs_inside_a_list_are_built():
    result = build_value([{"type": "point", "x": 1.0}, [2.0, 3.0]], "mixed")
    assert isinstance(result[0], Point)
    assert result[0].x == 1.0
    assert result[1] == [2.0, 3.0]


def test_missing_type_key_is_an_error_listing_types():
    with pytest.raises(ConfigError) as err:
        build_value({"x": 2.0, "y": 1.5}, "goal_pose")
    message = str(err.value)
    assert "goal_pose" in message
    assert "type" in message
    for tag in ("pose", "pose_cov", "poses", "path", "point", "quaternion", "duration"):
        assert tag in message


def test_unknown_type_is_an_error_listing_sorted_types():
    with pytest.raises(ConfigError) as err:
        build_value({"type": "poze", "x": 1.0}, "goal_pose")
    message = str(err.value)
    assert "goal_pose" in message
    assert "poze" in message
    listed = message.split("valid types:")[1]
    assert listed.strip().startswith("duration")  # sorted


def test_non_string_type_is_an_error():
    with pytest.raises(ConfigError) as err:
        build_value({"type": 7}, "goal_pose")
    assert "goal_pose" in str(err.value)


def test_unknown_key_in_spec_is_an_error():
    with pytest.raises(ConfigError) as err:
        build_value({"type": "pose", "fram_id": "map", "x": 1.0}, "goal_pose")
    message = str(err.value)
    assert "goal_pose" in message
    assert "fram_id" in message
    assert "frame_id" in message  # the allowed-keys list points at the fix


# ---------------------------------------------------------------------------
# load_config
# ---------------------------------------------------------------------------


FULL = """
ros:
  namespace: /robot1
  node_name: nav_brain
  use_sim_time: true
blackboard:
  goal_pose:    {type: pose, frame_id: map, x: 2.0, y: 1.5, yaw: 0.0}
  initial_pose: {type: pose_cov, frame_id: map, x: 0.0, y: 0.0, yaw: 0.0}
  waypoints:    {type: poses, frame_id: map, points: [[1.0, 2.0], [3.0, 4.0, 1.57]]}
  route:        {type: path, frame_id: map, points: [[0.0, 0.0], [1.0, 0.0]]}
  backup_dist:  0.15
  dock_id:      "dock_a"
"""


def test_load_config_full_file(tmp_path):
    config = load_config(write(tmp_path, FULL))
    assert isinstance(config, Nav2Config)
    assert config.ros == RosConfig(namespace="/robot1", node_name="nav_brain", use_sim_time=True)

    bb = config.blackboard
    assert isinstance(bb["goal_pose"], PoseStamped)
    assert bb["goal_pose"].pose.position.x == 2.0
    assert isinstance(bb["initial_pose"], PoseWithCovarianceStamped)
    assert isinstance(bb["waypoints"], list) and len(bb["waypoints"]) == 2
    assert isinstance(bb["route"], PathMsg) and len(bb["route"].poses) == 2
    assert bb["backup_dist"] == 0.15
    assert bb["dock_id"] == "dock_a"


def test_load_config_accepts_str_path(tmp_path):
    config = load_config(str(write(tmp_path, FULL)))
    assert config.ros.namespace == "/robot1"


def test_load_config_defaults_when_sections_absent(tmp_path):
    config = load_config(write(tmp_path, "other: 1\n"))
    assert config.ros == RosConfig()
    assert config.ros.namespace == ""
    assert config.ros.node_name == "bteng_nav2"
    assert config.ros.use_sim_time is False
    assert config.blackboard == {}


def test_load_config_empty_file_is_all_defaults(tmp_path):
    config = load_config(write(tmp_path, ""))
    assert config.ros == RosConfig()
    assert config.blackboard == {}
    assert config.raw == {}


def test_load_config_null_sections_are_defaults(tmp_path):
    config = load_config(write(tmp_path, "ros:\nblackboard:\n"))
    assert config.ros == RosConfig()
    assert config.blackboard == {}


def test_load_config_preserves_unknown_top_level_sections(tmp_path):
    text = "app:\n  mission: patrol\nros:\n  namespace: r2\nblackboard:\n  n: 3\n"
    config = load_config(write(tmp_path, text))
    assert config.raw["app"] == {"mission": "patrol"}
    assert config.raw["ros"] == {"namespace": "r2"}
    assert "app" not in config.blackboard
    assert config.blackboard == {"n": 3}


def test_load_config_missing_file_raises():
    with pytest.raises(ConfigError) as err:
        load_config("/nonexistent/dir/params.yaml")
    assert "not found" in str(err.value)
    assert "params.yaml" in str(err.value)


def test_load_config_directory_is_not_a_file(tmp_path):
    with pytest.raises(ConfigError):
        load_config(tmp_path)


def test_load_config_invalid_yaml_raises(tmp_path):
    with pytest.raises(ConfigError) as err:
        load_config(write(tmp_path, "ros: [unclosed\n"))
    assert "YAML" in str(err.value)


def test_load_config_non_mapping_top_level_raises(tmp_path):
    with pytest.raises(ConfigError):
        load_config(write(tmp_path, "- a\n- b\n"))


def test_load_config_error_names_the_blackboard_key(tmp_path):
    text = "blackboard:\n  goal_pose: {type: pose, fram_id: map}\n"
    with pytest.raises(ConfigError) as err:
        load_config(write(tmp_path, text))
    assert "goal_pose" in str(err.value)


def test_load_config_rejects_unknown_ros_key(tmp_path):
    with pytest.raises(ConfigError) as err:
        load_config(write(tmp_path, "ros:\n  use_simtime: true\n"))
    message = str(err.value)
    assert "ros" in message
    assert "use_simtime" in message


def test_load_config_rejects_non_bool_use_sim_time(tmp_path):
    with pytest.raises(ConfigError) as err:
        load_config(write(tmp_path, "ros:\n  use_sim_time: yes please\n"))
    assert "use_sim_time" in str(err.value)


def test_load_config_rejects_non_mapping_sections(tmp_path):
    with pytest.raises(ConfigError):
        load_config(write(tmp_path, "ros: hello\n"))
    with pytest.raises(ConfigError):
        load_config(write(tmp_path, "blackboard:\n  - a\n"))


@pytest.mark.parametrize(
    ("given", "expected"),
    [
        ("robot1", "/robot1"),
        ("/robot1", "/robot1"),
        ("/robot1/", "/robot1"),
        ("robot1/", "/robot1"),
        ('""', ""),
        ('"/"', ""),
        ('" /robot1 "', "/robot1"),
        ("/fleet/robot1", "/fleet/robot1"),
    ],
)
def test_namespace_normalization(tmp_path, given, expected):
    config = load_config(write(tmp_path, f"ros:\n  namespace: {given}\n"))
    assert config.ros.namespace == expected


def test_namespace_absent_stays_empty(tmp_path):
    assert load_config(write(tmp_path, "ros:\n  node_name: x\n")).ros.namespace == ""


# ---------------------------------------------------------------------------
# Nav2Config.empty / with_namespace
# ---------------------------------------------------------------------------


def test_nav2_config_empty_is_all_defaults():
    config = Nav2Config.empty()
    assert isinstance(config, Nav2Config)
    assert config.ros == RosConfig()
    assert config.ros.namespace == ""
    assert config.ros.node_name == "bteng_nav2"
    assert config.ros.use_sim_time is False
    assert config.blackboard == {}
    assert config.raw == {}


def test_nav2_config_empty_returns_independent_containers():
    """A shared default dict would leak one run's presets into the next."""
    first = Nav2Config.empty()
    second = Nav2Config.empty()
    first.blackboard["x"] = 1
    assert second.blackboard == {}
    assert first.raw is not second.raw


@pytest.mark.parametrize(
    ("given", "expected"),
    [
        ("robot1", "/robot1"),
        ("/robot1", "/robot1"),
        ("/robot1/", "/robot1"),
        ("", ""),
        ("/", ""),
        (" /robot1 ", "/robot1"),
        ("/fleet/robot1", "/fleet/robot1"),
    ],
)
def test_with_namespace_normalizes(given, expected):
    assert Nav2Config.empty().with_namespace(given).ros.namespace == expected


def test_with_namespace_overrides_file_value_and_keeps_the_rest(tmp_path):
    config = load_config(write(tmp_path, FULL))
    overridden = config.with_namespace("robot9")

    assert overridden.ros.namespace == "/robot9"
    assert overridden.ros.node_name == "nav_brain"
    assert overridden.ros.use_sim_time is True
    assert overridden.blackboard is config.blackboard  # messages are not rebuilt
    assert overridden.raw == config.raw
    assert config.ros.namespace == "/robot1"  # original untouched


def test_with_namespace_leaves_raw_reporting_the_file(tmp_path):
    config = load_config(write(tmp_path, "ros:\n  namespace: /robot1\n")).with_namespace("r2")
    assert config.ros.namespace == "/r2"
    assert config.raw["ros"]["namespace"] == "/robot1"


def test_with_namespace_rejects_non_string():
    with pytest.raises(ConfigError):
        Nav2Config.empty().with_namespace(7)  # type: ignore[arg-type]


def test_nav2_config_is_frozen(tmp_path):
    config = load_config(write(tmp_path, ""))
    with pytest.raises(Exception):
        config.ros = RosConfig()  # type: ignore[misc]


# ---------------------------------------------------------------------------
# parse_bb_presets
# ---------------------------------------------------------------------------


def test_parse_bb_presets_native_types():
    result = parse_bb_presets(["x=false", "n=5", "s=abc", "f=1.25", "none=null"])
    assert result == {"x": False, "n": 5, "s": "abc", "f": 1.25, "none": None}
    assert result["x"] is False
    assert isinstance(result["n"], int)


def test_parse_bb_presets_empty_list():
    assert parse_bb_presets([]) == {}


def test_parse_bb_presets_strips_whitespace():
    assert parse_bb_presets([" key = 7 "]) == {"key": 7}


def test_parse_bb_presets_value_may_contain_equals():
    assert parse_bb_presets(["expr=a=b"]) == {"expr": "a=b"}


def test_parse_bb_presets_builds_mapping_values():
    result = parse_bb_presets(["goal_pose={type: pose, x: 3.0}"])
    msg = result["goal_pose"]
    assert isinstance(msg, PoseStamped)
    assert msg.pose.position.x == 3.0
    assert msg.header.frame_id == "map"


def test_parse_bb_presets_bad_mapping_reports_key():
    with pytest.raises(ConfigError) as err:
        parse_bb_presets(["goal_pose={x: 3.0}"])
    assert "goal_pose" in str(err.value)


def test_parse_bb_presets_malformed_item_raises_value_error():
    with pytest.raises(ValueError) as err:
        parse_bb_presets(["justakey"])
    assert "KEY=VALUE" in str(err.value)


def test_parse_bb_presets_later_key_wins():
    assert parse_bb_presets(["n=1", "n=2"]) == {"n": 2}
