"""Typed params-file loader: YAML in, ROS messages out.

A BT XML carries strings only, but the nav2 nodes want real messages —
``NavigateToPoseNode.goal_pose`` is a ``geometry_msgs/PoseStamped``,
``SetInitialPoseNode.initial_pose`` a ``PoseWithCovarianceStamped``,
``FollowPathNode.path`` a ``nav_msgs/Path``.  There is no way to spell those in
an attribute, so the params file *builds* them here and the runtime seeds them
into the blackboard before the tree ticks; the XML then only says
``{goal_pose}``.  That is what lets a whole navigation app be a YAML file plus
an XML file with no application Python.

This is the nav2 analogue of the bteng_opcua node registry: a small typed
vocabulary (``type: pose``, ``type: path``, …) instead of a general
object-from-dict mechanism, so a typo fails at load time with the offending
blackboard key in the message rather than deep inside a ticking node.

Every message import is *inside* its builder.  The package must stay importable
with no ROS installed (the node modules do the same), so ``import
bteng_nav2.config`` costs nothing until a value is actually built.

``header.stamp`` is deliberately left at zero: there is no ROS clock at
config-load time, and both the nav2 servers and the tf2 lookups treat a zero
stamp as "now / latest available", which is what a static goal wants.
"""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field, replace
from pathlib import Path
from typing import Any, Callable, NoReturn

__all__ = [
    "BUILDERS",
    "ConfigError",
    "Nav2Config",
    "RosConfig",
    "build_value",
    "load_config",
    "parse_bb_presets",
]


class ConfigError(Exception):
    """A params file cannot be turned into messages.

    Raised instead of returning something half-built: a bad goal pose must stop
    the run at load time, not become a navigation command.
    """


@dataclass(frozen=True)
class RosConfig:
    """The ROS side of the params file.

    ``namespace`` is normalized to either ``""`` or ``/name`` so callers can
    concatenate it with a topic name without guessing about slashes.
    """

    namespace: str = ""
    node_name: str = "bteng_nav2"
    use_sim_time: bool = False


@dataclass(frozen=True)
class Nav2Config:
    """A loaded params file.

    ``blackboard`` values are already ROS messages — the runtime just seeds
    them.  ``raw`` is the untouched parse so a project can keep its own
    top-level sections in the same file and read them back.
    """

    ros: RosConfig = field(default_factory=RosConfig)
    blackboard: dict[str, Any] = field(default_factory=dict)
    raw: dict[str, Any] = field(default_factory=dict)

    @classmethod
    def empty(cls) -> Nav2Config:
        """An all-defaults config.

        The runtime needs *something* to run against when invoked with a tree but
        no params file; handing it this instead of ``None`` keeps every call site
        free of ``if config is not None`` branches.
        """
        return cls(ros=RosConfig(), blackboard={}, raw={})

    def with_namespace(self, namespace: str) -> Nav2Config:
        """Copy with ``ros.namespace`` replaced (normalized the same as the file).

        A CLI ``--namespace`` override must beat the file for the *same* params
        file — fleet deployments run one YAML per robot type, not per robot.
        ``raw`` is intentionally left alone: it is the file as read, and rewriting
        it would make it lie about the file's contents.
        """
        normalized = _normalize_namespace(namespace, "ros")
        return replace(self, ros=replace(self.ros, namespace=normalized))


# ── Error helpers ───────────────────────────────────────────────────────────────

def _fail(where: str, message: str) -> NoReturn:
    """Raise with the location baked in — an error naming only the problem
    ("unknown key 'fram_id'") is useless in a file with twenty poses."""
    raise ConfigError(f"config error at {where or '<value>'}: {message}")


def _valid_types() -> str:
    return ", ".join(sorted(BUILDERS))


# ── Scalar coercion ─────────────────────────────────────────────────────────────

def _check_keys(spec: Mapping[str, Any], allowed: frozenset[str], where: str) -> None:
    """Reject unknown keys.  A typo'd ``fram_id`` must not silently fall back to
    the default frame — that would put the goal in the wrong place at runtime."""
    unknown = sorted(str(k) for k in spec if str(k) not in allowed)
    if unknown:
        _fail(
            where,
            f"unknown key(s) {', '.join(repr(k) for k in unknown)}; "
            f"allowed: {', '.join(sorted(allowed))}",
        )


def _as_float(spec: Mapping[str, Any], key: str, where: str, default: float = 0.0) -> float:
    value = spec.get(key, default)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        _fail(where, f"{key!r} must be a number, got {value!r}")
    return float(value)


def _as_int(spec: Mapping[str, Any], key: str, where: str, default: int = 0) -> int:
    value = spec.get(key, default)
    if isinstance(value, bool) or not isinstance(value, int):
        _fail(where, f"{key!r} must be an integer, got {value!r}")
    return int(value)


def _as_str(spec: Mapping[str, Any], key: str, where: str, default: str) -> str:
    value = spec.get(key, default)
    if not isinstance(value, str):
        _fail(where, f"{key!r} must be a string, got {value!r}")
    return value


def _as_bool(value: Any, key: str, where: str) -> bool:
    if not isinstance(value, bool):
        _fail(where, f"{key!r} must be a boolean, got {value!r}")
    return value


# ── Orientation ─────────────────────────────────────────────────────────────────

_QUAT_KEYS = ("qx", "qy", "qz", "qw")


def yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    """Yaw-only rotation as (x, y, z, w).

    A ground robot's pose is a 2-D pose; asking a user for four quaternion
    components to say "face east" is how goal poses get typed wrong.
    """
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def _orientation(spec: Mapping[str, Any], where: str) -> tuple[float, float, float, float]:
    """Resolve ``yaw:`` or explicit ``qx/qy/qz/qw`` into (x, y, z, w)."""
    explicit = [k for k in _QUAT_KEYS if k in spec]
    if "yaw" in spec and explicit:
        _fail(
            where,
            f"'yaw' and explicit quaternion key(s) {', '.join(explicit)} are mutually "
            "exclusive; give one orientation, not two",
        )
    if not explicit:
        return yaw_to_quaternion(_as_float(spec, "yaw", where, 0.0))

    quat = (
        _as_float(spec, "qx", where, 0.0),
        _as_float(spec, "qy", where, 0.0),
        _as_float(spec, "qz", where, 0.0),
        _as_float(spec, "qw", where, 1.0),
    )
    norm = math.sqrt(sum(c * c for c in quat))
    if abs(norm - 1.0) > 1e-3:
        # Almost always a forgotten component (qz without qw); an unnormalized
        # quaternion is accepted silently by ROS and yields a skewed pose.
        _fail(where, f"quaternion {quat} is not normalized (norm={norm:.6f})")
    return quat


# ── Pose plumbing ───────────────────────────────────────────────────────────────

_POSE_KEYS = frozenset({"type", "frame_id", "x", "y", "z", "yaw", *_QUAT_KEYS})
_POSE_COV_KEYS = _POSE_KEYS | {"covariance"}
_POINTS_KEYS = frozenset({"type", "frame_id", "points"})
_POINT_KEYS = frozenset({"type", "x", "y", "z"})
_QUATERNION_KEYS = frozenset({"type", "yaw", *_QUAT_KEYS})
_DURATION_KEYS = frozenset({"type", "sec", "nanosec", "seconds"})


def _fill_pose(pose: Any, spec: Mapping[str, Any], where: str) -> None:
    """Write position/orientation onto an existing ``geometry_msgs/Pose``."""
    pose.position.x = _as_float(spec, "x", where, 0.0)
    pose.position.y = _as_float(spec, "y", where, 0.0)
    pose.position.z = _as_float(spec, "z", where, 0.0)
    qx, qy, qz, qw = _orientation(spec, where)
    pose.orientation.x = qx
    pose.orientation.y = qy
    pose.orientation.z = qz
    pose.orientation.w = qw


def _pose_stamped(spec: Mapping[str, Any], where: str, default_frame: str = "map") -> Any:
    from geometry_msgs.msg import PoseStamped

    msg = PoseStamped()
    msg.header.frame_id = _as_str(spec, "frame_id", where, default_frame)
    _fill_pose(msg.pose, spec, where)
    return msg


def _point_entry(entry: Any, frame_id: str, where: str) -> Any:
    """One ``points:`` entry -> PoseStamped.

    Accepts ``[x, y]``, ``[x, y, yaw]`` or a full pose mapping, because a
    waypoint list is usually all defaults and spelling out ten mappings is
    noise.  The parent's ``frame_id`` is inherited unless the entry overrides.
    """
    if isinstance(entry, Mapping):
        _check_keys(entry, _POSE_KEYS, where)
        tag = entry.get("type")
        if tag is not None and tag != "pose":
            _fail(where, f"a points entry may only be 'type: pose', got {tag!r}")
        return _pose_stamped(entry, where, default_frame=frame_id)

    if isinstance(entry, Sequence) and not isinstance(entry, (str, bytes)):
        if len(entry) not in (2, 3):
            _fail(where, f"a points entry must be [x, y] or [x, y, yaw], got {list(entry)!r}")
        keys = ("x", "y", "yaw")
        return _pose_stamped(dict(zip(keys, entry)), where, default_frame=frame_id)

    _fail(where, f"a points entry must be [x, y], [x, y, yaw] or a pose mapping, got {entry!r}")


def _point_list(spec: Mapping[str, Any], where: str) -> tuple[str, list[Any]]:
    frame_id = _as_str(spec, "frame_id", where, "map")
    points = spec.get("points", [])
    if not isinstance(points, Sequence) or isinstance(points, (str, bytes)):
        _fail(where, f"'points' must be a list, got {points!r}")
    poses = [
        _point_entry(entry, frame_id, f"{where}.points[{i}]") for i, entry in enumerate(points)
    ]
    return frame_id, poses


# ── Builders ────────────────────────────────────────────────────────────────────

def _build_pose(spec: Mapping[str, Any], where: str) -> Any:
    """``geometry_msgs/PoseStamped`` — the goal_pose / initial-pose workhorse."""
    _check_keys(spec, _POSE_KEYS, where)
    return _pose_stamped(spec, where)


def _build_pose_cov(spec: Mapping[str, Any], where: str) -> Any:
    """``geometry_msgs/PoseWithCovarianceStamped`` for AMCL's initial pose."""
    from geometry_msgs.msg import PoseWithCovarianceStamped

    _check_keys(spec, _POSE_COV_KEYS, where)
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = _as_str(spec, "frame_id", where, "map")
    _fill_pose(msg.pose.pose, spec, where)

    covariance = spec.get("covariance")
    if covariance is not None:
        if not isinstance(covariance, Sequence) or isinstance(covariance, (str, bytes)):
            _fail(where, f"'covariance' must be a list of 36 numbers, got {covariance!r}")
        if len(covariance) != 36:
            _fail(
                where,
                f"'covariance' must have exactly 36 entries (6x6 row-major), got {len(covariance)}",
            )
        values = []
        for i, item in enumerate(covariance):
            if isinstance(item, bool) or not isinstance(item, (int, float)):
                _fail(where, f"covariance[{i}] must be a number, got {item!r}")
            values.append(float(item))
        msg.pose.covariance = values
    return msg


def _build_poses(spec: Mapping[str, Any], where: str) -> list[Any]:
    """``list[PoseStamped]`` — NavigateThroughPoses/waypoint-follower input."""
    _check_keys(spec, _POINTS_KEYS, where)
    return _point_list(spec, where)[1]


def _build_path(spec: Mapping[str, Any], where: str) -> Any:
    """``nav_msgs/Path`` — a pre-planned route handed straight to FollowPath."""
    from nav_msgs.msg import Path as PathMsg

    _check_keys(spec, _POINTS_KEYS, where)
    frame_id, poses = _point_list(spec, where)
    msg = PathMsg()
    msg.header.frame_id = frame_id
    msg.poses = poses
    return msg


def _build_point(spec: Mapping[str, Any], where: str) -> Any:
    """``geometry_msgs/Point``."""
    from geometry_msgs.msg import Point

    _check_keys(spec, _POINT_KEYS, where)
    return Point(
        x=_as_float(spec, "x", where, 0.0),
        y=_as_float(spec, "y", where, 0.0),
        z=_as_float(spec, "z", where, 0.0),
    )


def _build_quaternion(spec: Mapping[str, Any], where: str) -> Any:
    """``geometry_msgs/Quaternion`` from ``yaw`` or explicit components."""
    from geometry_msgs.msg import Quaternion

    _check_keys(spec, _QUATERNION_KEYS, where)
    qx, qy, qz, qw = _orientation(spec, where)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)


def _build_duration(spec: Mapping[str, Any], where: str) -> Any:
    """``builtin_interfaces/Duration`` from ``sec``/``nanosec`` or ``seconds``.

    ``seconds: 1.5`` exists because a timeout is naturally written as a float;
    mixing it with the split form is rejected rather than guessed at.
    """
    from builtin_interfaces.msg import Duration

    _check_keys(spec, _DURATION_KEYS, where)
    if "seconds" in spec and ("sec" in spec or "nanosec" in spec):
        _fail(where, "'seconds' cannot be combined with 'sec'/'nanosec'; use one form")

    if "seconds" in spec:
        total = _as_float(spec, "seconds", where, 0.0)
        sec = int(total)
        nanosec = int(round((total - sec) * 1e9))
        if nanosec < 0:  # negative durations: keep nanosec non-negative
            sec -= 1
            nanosec += 1_000_000_000
        return Duration(sec=sec, nanosec=nanosec)

    return Duration(
        sec=_as_int(spec, "sec", where, 0),
        nanosec=_as_int(spec, "nanosec", where, 0),
    )


BUILDERS: dict[str, Callable[..., Any]] = {
    "pose": _build_pose,
    "pose_cov": _build_pose_cov,
    "poses": _build_poses,
    "path": _build_path,
    "point": _build_point,
    "quaternion": _build_quaternion,
    "duration": _build_duration,
}


# ── Public value building ───────────────────────────────────────────────────────

_SPEC_KEYS = {
    "pose": _POSE_KEYS,
    "pose_cov": _POSE_COV_KEYS,
    "poses": _POINTS_KEYS,
    "path": _POINTS_KEYS,
    "point": _POINT_KEYS,
    "quaternion": _QUATERNION_KEYS,
    "duration": _DURATION_KEYS,
}


def _validate_spec(tag: str, spec: Mapping[str, Any], where: str) -> None:
    """Run every check a builder would, without constructing the message.

    Keeps --dry-run honest: an unknown key, a yaw/quaternion conflict, an
    unnormalized quaternion, a wrong-length covariance or a non-numeric point
    is caught on a machine with no ROS, which is exactly where CI runs.
    """
    _check_keys(spec, _SPEC_KEYS[tag], where)

    if tag in ("pose", "pose_cov", "quaternion"):
        _orientation(spec, where)
    if tag in ("pose", "pose_cov", "point"):
        for axis in ("x", "y", "z"):
            _as_float(spec, axis, where, 0.0)
    if tag == "pose_cov":
        covariance = spec.get("covariance")
        if covariance is not None:
            if not isinstance(covariance, Sequence) or isinstance(covariance, (str, bytes)):
                _fail(where, f"'covariance' must be a list of 36 numbers, got {covariance!r}")
            if len(covariance) != 36:
                _fail(
                    where,
                    f"'covariance' must have exactly 36 entries (6x6 row-major), "
                    f"got {len(covariance)}",
                )
            for i, item in enumerate(covariance):
                if isinstance(item, bool) or not isinstance(item, (int, float)):
                    _fail(where, f"covariance[{i}] must be a number, got {item!r}")
    if tag in ("poses", "path"):
        points = spec.get("points")
        if points is None:
            _fail(where, "'points' is required: a list of [x, y], [x, y, yaw] or pose mappings")
        if not isinstance(points, Sequence) or isinstance(points, (str, bytes)):
            _fail(where, f"'points' must be a list, got {points!r}")
        for i, entry in enumerate(points):
            spot = f"{where}.points[{i}]"
            if isinstance(entry, Mapping):
                _check_keys(entry, _POSE_KEYS, spot)
                _orientation(entry, spot)
            elif isinstance(entry, Sequence) and not isinstance(entry, (str, bytes)):
                if not 2 <= len(entry) <= 3:
                    _fail(spot, f"expected [x, y] or [x, y, yaw], got {len(entry)} values")
                for value in entry:
                    if isinstance(value, bool) or not isinstance(value, (int, float)):
                        _fail(spot, f"expected numbers, got {value!r}")
            else:
                _fail(spot, f"expected a list or a pose mapping, got {entry!r}")
    if tag == "duration":
        if "seconds" in spec and ("sec" in spec or "nanosec" in spec):
            _fail(where, "give either 'seconds' or 'sec'/'nanosec', not both")


class UnbuiltMessage:
    """Stand-in for a message that was validated but deliberately not built.

    ``--dry-run`` is the CI gate: it must work on a machine with no ROS at all.
    Building a real geometry_msgs/PoseStamped there fails at import, so the spec
    is validated (keys, types, conflicts) and this is returned instead. It knows
    what it *would* have built, which is what the dry-run report prints.
    """

    __slots__ = ("type_tag", "fields")

    def __init__(self, type_tag: str, fields: Mapping[str, Any]) -> None:
        self.type_tag = type_tag
        self.fields = dict(fields)

    def __repr__(self) -> str:
        return f"<unbuilt {self.type_tag}>"


def build_value(spec: Any, where: str = "", build: bool = True) -> Any:
    """Turn one params value into a blackboard value.

    ``build=False`` validates the spec without constructing the ROS message, so
    the whole params file can be checked where the message packages are not
    importable.

    A mapping is always a message spec: it must carry ``type:``.  Passing an
    untyped mapping through would turn a mistyped tag into a failure deep inside
    a ticking node, long after the file that caused it was read.  Everything
    else — scalars, ``None``, bools, lists of scalars — is already a blackboard
    value and is returned unchanged.
    """
    if isinstance(spec, Mapping):
        if "type" not in spec:
            _fail(
                where,
                f"mapping has no 'type:' key, so there is nothing to build; "
                f"valid types: {_valid_types()}",
            )
        tag = spec["type"]
        builder = BUILDERS.get(tag) if isinstance(tag, str) else None
        if builder is None:
            _fail(where, f"unknown type {tag!r}; valid types: {_valid_types()}")
        if not build:
            _validate_spec(tag, spec, where)
            return UnbuiltMessage(tag, spec)
        return builder(spec, where)

    if isinstance(spec, list) and any(isinstance(item, (Mapping, list)) for item in spec):
        # Nested specs are built in place; a plain list of scalars is returned
        # as-is (identity preserved) so nothing surprises a caller comparing it.
        return [build_value(item, f"{where}[{i}]", build) for i, item in enumerate(spec)]

    return spec


# ── File loading ────────────────────────────────────────────────────────────────

_ROS_KEYS = frozenset({"namespace", "node_name", "use_sim_time"})


def _normalize_namespace(value: str, where: str) -> str:
    """``robot1`` -> ``/robot1``, ``/robot1/`` -> ``/robot1``, ``""`` stays ``""``.

    Callers build topic names by concatenation, so the shape has to be settled
    once here instead of at every use site.
    """
    if not isinstance(value, str):
        _fail(where, f"'namespace' must be a string, got {value!r}")
    namespace = value.strip().rstrip("/")
    if not namespace:
        return ""
    return namespace if namespace.startswith("/") else "/" + namespace


def _load_ros(raw: Any) -> RosConfig:
    if raw is None:
        return RosConfig()
    if not isinstance(raw, Mapping):
        _fail("ros", f"section must be a mapping, got {raw!r}")
    _check_keys(raw, _ROS_KEYS, "ros")
    defaults = RosConfig()
    return RosConfig(
        namespace=_normalize_namespace(raw.get("namespace", defaults.namespace), "ros"),
        node_name=_as_str(raw, "node_name", "ros", defaults.node_name),
        use_sim_time=_as_bool(
            raw.get("use_sim_time", defaults.use_sim_time), "use_sim_time", "ros"
        ),
    )


def _load_blackboard(raw: Any, build: bool = True) -> dict[str, Any]:
    if raw is None:
        return {}
    if not isinstance(raw, Mapping):
        _fail("blackboard", f"section must be a mapping of KEY: VALUE, got {raw!r}")
    return {
        str(key): build_value(value, f"blackboard.{key}", build) for key, value in raw.items()
    }


def load_config(path: str | Path, build_messages: bool = True) -> Nav2Config:
    """Read a params YAML file and build its ``blackboard:`` values.

    ``build_messages=False`` validates every spec without constructing the ROS
    messages, so ``--dry-run`` works with no ROS installed at all.

    Unknown *top-level* sections are kept in ``raw`` and otherwise ignored: a
    project may keep launch or app settings in the same file.  Unknown keys
    *inside* a known section are errors, since there they can only be typos.
    """
    import yaml

    file = Path(path)
    if not file.is_file():
        _fail(str(file), "config file not found")

    try:
        parsed = yaml.safe_load(file.read_text(encoding="utf-8"))
    except yaml.YAMLError as exc:
        _fail(str(file), f"invalid YAML: {exc}")
    except OSError as exc:
        _fail(str(file), f"cannot read config file: {exc}")

    if parsed is None:  # empty file is a valid, all-defaults config
        parsed = {}
    if not isinstance(parsed, Mapping):
        _fail(str(file), f"top level must be a mapping of sections, got {type(parsed).__name__}")

    raw = dict(parsed)
    return Nav2Config(
        ros=_load_ros(raw.get("ros")),
        blackboard=_load_blackboard(raw.get("blackboard"), build_messages),
        raw=raw,
    )


def parse_bb_presets(raw: list[str]) -> dict[str, Any]:
    """Parse repeated ``KEY=VALUE`` strings (a CLI ``--bb`` flag) into a dict.

    Values go through a YAML scalar pass so they carry native types, the same as
    the ``blackboard:`` section: ``x=false`` -> ``False``, ``n=5`` -> ``5``,
    ``s=abc`` -> ``"abc"``.  Without it every preset would be a string and
    ``--bb enabled=false`` would write ``bool("false")`` == True.  A mapping
    value is run through :func:`build_value`, so
    ``--bb 'goal_pose={type: pose, x: 3.0}'`` builds a real PoseStamped.
    """
    import yaml

    result: dict[str, Any] = {}
    for item in raw:
        if "=" not in item:
            raise ValueError(f"blackboard preset must be KEY=VALUE, got: {item!r}")
        key, _, value = item.partition("=")
        key = key.strip()
        try:
            parsed = yaml.safe_load(value.strip())
        except yaml.YAMLError:
            parsed = value.strip()
        result[key] = build_value(parsed, f"--bb {key}")
    return result
