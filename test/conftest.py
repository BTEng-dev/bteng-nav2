"""
conftest.py for bteng_nav2 tests.

Stubs all ROS 2 message, service, and action types (rclpy, nav2_msgs,
geometry_msgs, std_msgs, nav_msgs, sensor_msgs, builtin_interfaces, and
opennav_docking_msgs) so the test suite runs without a ROS installation.
"""

import sys
import types

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_module(*parts):
    """Return a fresh ModuleType, register every prefix, and set as attribute on parent."""
    name = ".".join(parts)
    mod = types.ModuleType(name)
    for i in range(1, len(parts) + 1):
        pkg = ".".join(parts[:i])
        if pkg not in sys.modules:
            sys.modules[pkg] = types.ModuleType(pkg)
    sys.modules[name] = mod
    # Make the submodule accessible as an attribute on its parent so that
    # unittest.mock.patch("pkg.sub.Class") resolves correctly on all Python versions.
    if len(parts) > 1:
        setattr(sys.modules[".".join(parts[:-1])], parts[-1], mod)
    return mod


# ---------------------------------------------------------------------------
# 1. rclpy mocking  (replicated from bteng_ros2/test/conftest.py pattern)
# ---------------------------------------------------------------------------

def _mock_rclpy():
    rclpy = _make_module("rclpy")
    rclpy.init = lambda *a, **kw: None
    rclpy.shutdown = lambda *a, **kw: None
    rclpy.ok = lambda *a, **kw: True
    rclpy.spin = lambda *a, **kw: None
    rclpy.spin_once = lambda *a, **kw: None
    rclpy.spin_until_future_complete = lambda *a, **kw: None

    # rclpy.node
    node_mod = _make_module("rclpy", "node")

    class Node:
        def __init__(self, name, *args, **kwargs):
            self._name = name

        def get_name(self):
            return self._name

        def get_logger(self):
            import logging
            return logging.getLogger(self._name)

        def create_publisher(self, *a, **kw):
            return _Publisher()

        def create_subscription(self, *a, **kw):
            return None

        def create_timer(self, *a, **kw):
            return None

        def create_client(self, *a, **kw):
            return _ServiceClient()

        def create_service(self, *a, **kw):
            return None

        def destroy_node(self):
            pass

        def destroy_publisher(self, *a, **kw):
            pass

        def destroy_subscription(self, *a, **kw):
            pass

        def destroy_client(self, *a, **kw):
            pass

        def declare_parameter(self, name, value=None, *a, **kw):
            return _Parameter(name, value)

        def get_parameter(self, name):
            return _Parameter(name)

        def set_parameters(self, *a, **kw):
            pass

        def executor(self):
            return None

    node_mod.Node = Node
    rclpy.node = node_mod

    # rclpy.action
    action_mod = _make_module("rclpy", "action")

    class ActionClient:
        def __init__(self, node, action_type, action_name, *a, **kw):
            pass

        def wait_for_server(self, timeout_sec=None):
            return True

        def send_goal_async(self, goal, *a, **kw):
            return _Future()

        def send_goal(self, goal, *a, **kw):
            return _GoalHandle()

    class ActionServer:
        def __init__(self, *a, **kw):
            pass

        def destroy(self):
            pass

    action_mod.ActionClient = ActionClient
    action_mod.ActionServer = ActionServer
    rclpy.action = action_mod

    # rclpy.task / Future
    task_mod = _make_module("rclpy", "task")

    class Future:
        def __init__(self):
            self._done = False
            self._result = None
            self._callbacks = []

        def done(self):
            return self._done

        def result(self):
            return self._result

        def add_done_callback(self, cb):
            self._callbacks.append(cb)

        def set_result(self, result):
            self._result = result
            self._done = True
            for cb in self._callbacks:
                cb(self)

    task_mod.Future = Future
    rclpy.task = task_mod

    # rclpy.callback_groups
    cb_mod = _make_module("rclpy", "callback_groups")

    class ReentrantCallbackGroup:
        pass

    class MutuallyExclusiveCallbackGroup:
        pass

    cb_mod.ReentrantCallbackGroup = ReentrantCallbackGroup
    cb_mod.MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup
    rclpy.callback_groups = cb_mod

    # rclpy.executors
    exec_mod = _make_module("rclpy", "executors")

    class SingleThreadedExecutor:
        def __init__(self, *a, **kw):
            pass

        def add_node(self, *a, **kw):
            pass

        def spin_once(self, *a, **kw):
            pass

        def spin(self):
            pass

        def shutdown(self):
            pass

    class MultiThreadedExecutor(SingleThreadedExecutor):
        pass

    exec_mod.SingleThreadedExecutor = SingleThreadedExecutor
    exec_mod.MultiThreadedExecutor = MultiThreadedExecutor
    rclpy.executors = exec_mod

    # rclpy.qos
    qos_mod = _make_module("rclpy", "qos")

    class QoSProfile:
        def __init__(self, *a, **kw):
            pass

    qos_mod.QoSProfile = QoSProfile
    qos_mod.qos_profile_system_default = QoSProfile()
    qos_mod.qos_profile_sensor_data = QoSProfile()
    qos_mod.qos_profile_services_default = QoSProfile()
    qos_mod.QoSDurabilityPolicy = types.SimpleNamespace(
        TRANSIENT_LOCAL=1, VOLATILE=2, SYSTEM_DEFAULT=3
    )
    qos_mod.QoSReliabilityPolicy = types.SimpleNamespace(
        RELIABLE=1, BEST_EFFORT=2, SYSTEM_DEFAULT=3
    )
    qos_mod.QoSHistoryPolicy = types.SimpleNamespace(
        KEEP_LAST=1, KEEP_ALL=2, SYSTEM_DEFAULT=3
    )
    rclpy.qos = qos_mod

    # rclpy.parameter
    param_mod = _make_module("rclpy", "parameter")

    class Parameter:
        def __init__(self, name, value=None):
            self.name = name
            self.value = value

        def get_parameter_value(self):
            return types.SimpleNamespace(
                string_value=str(self.value) if self.value is not None else "",
                integer_value=int(self.value) if isinstance(self.value, int) else 0,
                double_value=float(self.value) if isinstance(self.value, (int, float)) else 0.0,
                bool_value=bool(self.value) if isinstance(self.value, bool) else False,
                type=0,
            )

    param_mod.Parameter = Parameter
    rclpy.parameter = param_mod

    # rclpy.time
    time_mod = _make_module("rclpy", "time")

    class Time:
        def __init__(self, seconds=0, nanoseconds=0, clock_type=None):
            self.seconds = seconds
            self.nanoseconds = nanoseconds

        def to_msg(self):
            return types.SimpleNamespace(sec=int(self.seconds), nanosec=int(self.nanoseconds))

        def nanoseconds(self):
            return int(self.seconds * 1e9) + self.nanoseconds

    class Duration:
        def __init__(self, seconds=0, nanoseconds=0):
            self.seconds = seconds
            self.nanoseconds_val = nanoseconds

        def to_msg(self):
            return types.SimpleNamespace(sec=int(self.seconds), nanosec=int(self.nanoseconds_val))

    class Clock:
        def __init__(self, *a, **kw):
            pass

        def now(self):
            return Time()

    time_mod.Time = Time
    time_mod.Duration = Duration
    time_mod.Clock = Clock
    rclpy.time = time_mod

    # rclpy.duration
    dur_mod = _make_module("rclpy", "duration")
    dur_mod.Duration = Duration
    rclpy.duration = dur_mod

    # rclpy.clock
    clock_mod = _make_module("rclpy", "clock")
    clock_mod.Clock = Clock
    rclpy.clock = clock_mod

    # rclpy.logging
    log_mod = _make_module("rclpy", "logging")
    log_mod.get_logger = lambda name="": __import__("logging").getLogger(name)
    rclpy.logging = log_mod

    # rclpy.lifecycle
    lifecycle_mod = _make_module("rclpy", "lifecycle")
    lifecycle_node_mod = _make_module("rclpy", "lifecycle", "node")

    class LifecycleNode(node_mod.Node):
        pass

    class TransitionCallbackReturn:
        SUCCESS = "success"
        FAILURE = "failure"
        ERROR = "error"

    lifecycle_mod.LifecycleNode = LifecycleNode
    lifecycle_mod.TransitionCallbackReturn = TransitionCallbackReturn
    lifecycle_node_mod.LifecycleNode = LifecycleNode
    lifecycle_node_mod.LifecycleState = type("LifecycleState", (), {})
    lifecycle_node_mod.TransitionCallbackReturn = TransitionCallbackReturn
    rclpy.lifecycle = lifecycle_mod

    # action_msgs
    action_msgs_mod = _make_module("action_msgs")
    action_msgs_msg_mod = _make_module("action_msgs", "msg")

    class GoalStatus:
        STATUS_UNKNOWN = 0
        STATUS_ACCEPTED = 1
        STATUS_EXECUTING = 2
        STATUS_CANCELING = 3
        STATUS_SUCCEEDED = 4
        STATUS_CANCELED = 5
        STATUS_ABORTED = 6

    action_msgs_msg_mod.GoalStatus = GoalStatus
    action_msgs_mod.msg = action_msgs_msg_mod

    sys.modules["rclpy"] = rclpy


# ---------------------------------------------------------------------------
# Private helpers used by stubs above
# ---------------------------------------------------------------------------

class _Publisher:
    def publish(self, msg):
        pass

    def destroy(self):
        pass


class _ServiceClient:
    def call(self, req):
        return None

    def call_async(self, req):
        return _Future()

    def wait_for_service(self, timeout_sec=None):
        return True


class _Future:
    def __init__(self):
        self._done = False
        self._result = None

    def done(self):
        return self._done

    def result(self):
        return self._result

    def add_done_callback(self, cb):
        pass


class _GoalHandle:
    def __init__(self):
        self.accepted = True

    def get_result_async(self):
        return _Future()


class _Parameter:
    def __init__(self, name, value=None):
        self.name = name
        self.value = value

    def get_parameter_value(self):
        return types.SimpleNamespace(
            string_value=str(self.value) if self.value is not None else "",
            integer_value=int(self.value) if isinstance(self.value, int) else 0,
            double_value=float(self.value) if isinstance(self.value, (int, float)) else 0.0,
            bool_value=bool(self.value) if isinstance(self.value, bool) else False,
            type=0,
        )


# ---------------------------------------------------------------------------
# 2. nav2_msgs mocking
# ---------------------------------------------------------------------------

def _mock_nav2_msgs():
    # -----------------------------------------------------------------------
    # Shared primitive stubs (referenced by multiple message types)
    # -----------------------------------------------------------------------

    class _Duration:
        def __init__(self, sec=0, nanosec=0):
            self.sec = sec
            self.nanosec = nanosec

    class _Header:
        def __init__(self, stamp=None, frame_id=""):
            self.stamp = stamp
            self.frame_id = frame_id

    class _Point:
        def __init__(self, x=0.0, y=0.0, z=0.0):
            self.x = x
            self.y = y
            self.z = z

    class _Quaternion:
        def __init__(self, x=0.0, y=0.0, z=0.0, w=1.0):
            self.x = x
            self.y = y
            self.z = z
            self.w = w

    class _Pose:
        def __init__(self):
            self.position = _Point()
            self.orientation = _Quaternion()

    class _PoseStamped:
        def __init__(self):
            self.header = _Header()
            self.pose = _Pose()

    class _PoseWithCovariance:
        def __init__(self):
            self.pose = _Pose()
            self.covariance = []

    class _PoseWithCovarianceStamped:
        def __init__(self):
            self.header = _Header()
            self.pose = _PoseWithCovariance()

    class _Path:
        def __init__(self):
            self.header = _Header()
            self.poses = []

    class _BasicResult:
        def __init__(self):
            self.error_code = 0

    class _OccupancyGrid:
        def __init__(self):
            self.header = _Header()
            self.info = None
            self.data = []

    class _Costmap:
        def __init__(self):
            self.header = _Header()
            self.metadata = None
            self.data = []

    # -----------------------------------------------------------------------
    # nav2_msgs.action stubs
    # -----------------------------------------------------------------------

    action_mod = _make_module("nav2_msgs", "action")

    # NavigateToPose
    class NavigateToPose:
        class Goal:
            def __init__(self):
                self.pose = _PoseStamped()
                self.behavior_tree = ""

        class Result:
            def __init__(self):
                self.result = _BasicResult()

        class Feedback:
            def __init__(self):
                self.current_pose = _PoseStamped()
                self.navigation_time = _Duration()
                self.estimated_time_remaining = _Duration()
                self.number_of_recoveries = 0
                self.distance_remaining = 0.0

    action_mod.NavigateToPose = NavigateToPose

    # NavigateThroughPoses
    class NavigateThroughPoses:
        class Goal:
            def __init__(self):
                self.poses = []
                self.behavior_tree = ""

        class Result:
            def __init__(self):
                self.result = _BasicResult()

        class Feedback:
            def __init__(self):
                self.current_pose = _PoseStamped()
                self.navigation_time = _Duration()
                self.estimated_time_remaining = _Duration()
                self.number_of_recoveries = 0
                self.distance_remaining = 0.0
                self.number_of_poses_remaining = 0

    action_mod.NavigateThroughPoses = NavigateThroughPoses

    # FollowPath
    class FollowPath:
        class Goal:
            def __init__(self):
                self.path = _Path()
                self.controller_id = ""
                self.goal_checker_id = ""

        class Result:
            def __init__(self):
                pass

        class Feedback:
            def __init__(self):
                self.distance_to_goal = 0.0
                self.speed = 0.0

    action_mod.FollowPath = FollowPath

    # ComputePathToPose
    class ComputePathToPose:
        class Goal:
            def __init__(self):
                self.goal = _PoseStamped()
                self.start = None
                self.planner_id = ""
                self.use_start = False

        class Result:
            def __init__(self):
                self.path = _Path()
                self.planning_time = _Duration()
                self.error_code = 0

        class Feedback:
            def __init__(self):
                pass

    action_mod.ComputePathToPose = ComputePathToPose

    # ComputePathThroughPoses
    class ComputePathThroughPoses:
        class Goal:
            def __init__(self):
                self.goals = []
                self.start = None
                self.planner_id = ""
                self.use_start = False

        class Result:
            def __init__(self):
                self.path = _Path()
                self.planning_time = _Duration()
                self.error_code = 0

        class Feedback:
            def __init__(self):
                pass

    action_mod.ComputePathThroughPoses = ComputePathThroughPoses

    # SmoothPath
    class SmoothPath:
        class Goal:
            def __init__(self):
                self.path = _Path()
                self.smoother_id = ""
                self.max_smoothing_duration = _Duration(sec=10)
                self.check_for_collisions = False

        class Result:
            def __init__(self):
                self.path = _Path()
                self.smoothing_duration = _Duration()
                self.was_completed = True
                self.error_code = 0

        class Feedback:
            def __init__(self):
                pass

    action_mod.SmoothPath = SmoothPath

    # Spin
    class Spin:
        class Goal:
            def __init__(self):
                self.target_yaw = 0.0
                self.time_allowance = _Duration(sec=10)

        class Result:
            def __init__(self):
                self.total_elapsed_time = _Duration()
                self.error_code = 0

        class Feedback:
            def __init__(self):
                self.angular_distance_traveled = 0.0

    action_mod.Spin = Spin

    # BackUp
    class BackUp:
        class Goal:
            def __init__(self):
                self.target = types.SimpleNamespace(x=0.15, y=0.0, z=0.0)
                self.speed = 0.025
                self.time_allowance = _Duration(sec=10)

        class Result:
            def __init__(self):
                self.total_elapsed_time = _Duration()
                self.error_code = 0

        class Feedback:
            def __init__(self):
                self.distance_traveled = 0.0

    action_mod.BackUp = BackUp

    # DriveOnHeading
    class DriveOnHeading:
        class Goal:
            def __init__(self):
                self.target = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
                self.speed = 0.0
                self.time_allowance = _Duration(sec=10)

        class Result:
            def __init__(self):
                self.total_elapsed_time = _Duration()
                self.error_code = 0

        class Feedback:
            def __init__(self):
                self.distance_traveled = 0.0

    action_mod.DriveOnHeading = DriveOnHeading

    # Wait
    class Wait:
        class Goal:
            def __init__(self):
                self.time = _Duration(sec=0)

        class Result:
            def __init__(self):
                self.total_elapsed_time = _Duration()

        class Feedback:
            def __init__(self):
                self.time_left = _Duration()

    action_mod.Wait = Wait

    # AssistedTeleop
    class AssistedTeleop:
        class Goal:
            def __init__(self):
                self.time_allowance = _Duration(sec=30)

        class Result:
            def __init__(self):
                self.total_elapsed_time = _Duration()

        class Feedback:
            def __init__(self):
                self.current_teleop_duration = _Duration()

    action_mod.AssistedTeleop = AssistedTeleop

    # -----------------------------------------------------------------------
    # nav2_msgs.srv stubs
    # -----------------------------------------------------------------------

    srv_mod = _make_module("nav2_msgs", "srv")

    # ClearEntireCostmap
    class ClearEntireCostmap:
        class Request:
            def __init__(self):
                pass

        class Response:
            def __init__(self):
                pass

    srv_mod.ClearEntireCostmap = ClearEntireCostmap

    # ClearCostmapAroundRobot
    class ClearCostmapAroundRobot:
        class Request:
            def __init__(self):
                self.reset_distance = 1.0

        class Response:
            def __init__(self):
                pass

    srv_mod.ClearCostmapAroundRobot = ClearCostmapAroundRobot

    # GetCostmap
    class GetCostmap:
        class Request:
            def __init__(self):
                pass

        class Response:
            def __init__(self):
                self.map = _OccupancyGrid()

    srv_mod.GetCostmap = GetCostmap

    # IsPathValid
    class IsPathValid:
        class Request:
            def __init__(self):
                self.path = _Path()

        class Response:
            def __init__(self):
                self.is_valid = True

    srv_mod.IsPathValid = IsPathValid

    # LoadMap
    class LoadMap:
        class Request:
            def __init__(self):
                self.map_url = ""

        class Response:
            def __init__(self):
                self.result = 0

    srv_mod.LoadMap = LoadMap

    # ManageLifecycleNodes
    class ManageLifecycleNodes:
        STARTUP = 0
        PAUSE = 1
        RESUME = 2
        RESET = 3
        SHUTDOWN = 4
        DESTROY = 5

        class Request:
            def __init__(self):
                self.command = 0

        class Response:
            def __init__(self):
                self.success = True

    srv_mod.ManageLifecycleNodes = ManageLifecycleNodes

    # SaveMap
    class SaveMap:
        class Request:
            def __init__(self):
                self.map_topic = "/map"
                self.map_url = ""
                self.free_thresh = 0.25
                self.occupied_thresh = 0.65
                self.image_format = "png"
                self.map_mode = "trinary"

        class Response:
            def __init__(self):
                self.result = True

    srv_mod.SaveMap = SaveMap

    # -----------------------------------------------------------------------
    # nav2_msgs.msg stubs
    # -----------------------------------------------------------------------

    msg_mod = _make_module("nav2_msgs", "msg")
    msg_mod.Costmap = _Costmap

    # -----------------------------------------------------------------------
    # Store primitives on action_mod so tests can reach them if needed
    # -----------------------------------------------------------------------

    action_mod._PoseStamped = _PoseStamped
    action_mod._Path = _Path
    action_mod._BasicResult = _BasicResult
    action_mod._Duration = _Duration


# ---------------------------------------------------------------------------
# 3. geometry_msgs mocking
# ---------------------------------------------------------------------------

def _mock_geometry_msgs():
    msg_mod = _make_module("geometry_msgs", "msg")

    class Header:
        def __init__(self, stamp=None, frame_id=""):
            self.stamp = stamp
            self.frame_id = frame_id

    class Point:
        def __init__(self, x=0.0, y=0.0, z=0.0):
            self.x = x
            self.y = y
            self.z = z

    class Quaternion:
        def __init__(self, x=0.0, y=0.0, z=0.0, w=1.0):
            self.x = x
            self.y = y
            self.z = z
            self.w = w

    class Pose:
        def __init__(self):
            self.position = Point()
            self.orientation = Quaternion()

    class PoseStamped:
        def __init__(self):
            self.header = Header()
            self.pose = Pose()

    class PoseWithCovariance:
        def __init__(self):
            self.pose = Pose()
            self.covariance = []

    class PoseWithCovarianceStamped:
        def __init__(self):
            self.header = Header()
            self.pose = PoseWithCovariance()

    class Vector3:
        def __init__(self, x=0.0, y=0.0, z=0.0):
            self.x = x
            self.y = y
            self.z = z

    class Twist:
        def __init__(self):
            self.linear = Vector3()
            self.angular = Vector3()

    class TwistWithCovariance:
        def __init__(self):
            self.twist = Twist()
            self.covariance = []

    class TwistStamped:
        def __init__(self):
            self.header = Header()
            self.twist = Twist()

    class Transform:
        def __init__(self):
            self.translation = Vector3()
            self.rotation = Quaternion()

    class TransformStamped:
        def __init__(self):
            self.header = Header()
            self.child_frame_id = ""
            self.transform = Transform()

    msg_mod.Header = Header
    msg_mod.Point = Point
    msg_mod.Quaternion = Quaternion
    msg_mod.Pose = Pose
    msg_mod.PoseStamped = PoseStamped
    msg_mod.PoseWithCovariance = PoseWithCovariance
    msg_mod.PoseWithCovarianceStamped = PoseWithCovarianceStamped
    msg_mod.Vector3 = Vector3
    msg_mod.Twist = Twist
    msg_mod.TwistWithCovariance = TwistWithCovariance
    msg_mod.TwistStamped = TwistStamped
    msg_mod.Transform = Transform
    msg_mod.TransformStamped = TransformStamped


# ---------------------------------------------------------------------------
# 4. std_msgs mocking
# ---------------------------------------------------------------------------

def _mock_std_msgs():
    msg_mod = _make_module("std_msgs", "msg")

    class Header:
        def __init__(self, stamp=None, frame_id=""):
            self.stamp = stamp
            self.frame_id = frame_id

    class String:
        def __init__(self, data=""):
            self.data = data

    class Bool:
        def __init__(self, data=False):
            self.data = data

    class Int32:
        def __init__(self, data=0):
            self.data = data

    class Float32:
        def __init__(self, data=0.0):
            self.data = data

    class Float64:
        def __init__(self, data=0.0):
            self.data = data

    class Empty:
        def __init__(self):
            pass

    msg_mod.Header = Header
    msg_mod.String = String
    msg_mod.Bool = Bool
    msg_mod.Int32 = Int32
    msg_mod.Float32 = Float32
    msg_mod.Float64 = Float64
    msg_mod.Empty = Empty


# ---------------------------------------------------------------------------
# 5. nav_msgs mocking
# ---------------------------------------------------------------------------

def _mock_nav_msgs():
    msg_mod = _make_module("nav_msgs", "msg")

    class _Header:
        def __init__(self, stamp=None, frame_id=""):
            self.stamp = stamp
            self.frame_id = frame_id

    class _Point:
        def __init__(self, x=0.0, y=0.0, z=0.0):
            self.x = x
            self.y = y
            self.z = z

    class _Quaternion:
        def __init__(self, x=0.0, y=0.0, z=0.0, w=1.0):
            self.x = x
            self.y = y
            self.z = z
            self.w = w

    class _Pose:
        def __init__(self):
            self.position = _Point()
            self.orientation = _Quaternion()

    class _PoseStamped:
        def __init__(self):
            self.header = _Header()
            self.pose = _Pose()

    class _Vector3:
        def __init__(self, x=0.0, y=0.0, z=0.0):
            self.x = x
            self.y = y
            self.z = z

    class _Twist:
        def __init__(self):
            self.linear = _Vector3()
            self.angular = _Vector3()

    class _TwistWithCovariance:
        def __init__(self):
            self.twist = _Twist()
            self.covariance = []

    class _PoseWithCovariance:
        def __init__(self):
            self.pose = _Pose()
            self.covariance = []

    class Path:
        def __init__(self):
            self.header = _Header()
            self.poses = []

    class Odometry:
        def __init__(self):
            self.header = _Header()
            self.child_frame_id = ""
            self.pose = _PoseWithCovariance()
            self.twist = _TwistWithCovariance()

    class OccupancyGrid:
        def __init__(self):
            self.header = _Header()
            self.info = None
            self.data = []

    class MapMetaData:
        def __init__(self):
            self.map_load_time = None
            self.resolution = 0.0
            self.width = 0
            self.height = 0
            self.origin = _Pose()

    msg_mod.Path = Path
    msg_mod.Odometry = Odometry
    msg_mod.OccupancyGrid = OccupancyGrid
    msg_mod.MapMetaData = MapMetaData


# ---------------------------------------------------------------------------
# 6. sensor_msgs mocking
# ---------------------------------------------------------------------------

def _mock_sensor_msgs():
    msg_mod = _make_module("sensor_msgs", "msg")

    class _Header:
        def __init__(self, stamp=None, frame_id=""):
            self.stamp = stamp
            self.frame_id = frame_id

    class LaserScan:
        def __init__(self):
            self.header = _Header()
            self.angle_min = 0.0
            self.angle_max = 0.0
            self.angle_increment = 0.0
            self.time_increment = 0.0
            self.scan_time = 0.0
            self.range_min = 0.0
            self.range_max = 0.0
            self.ranges = []
            self.intensities = []

    class PointCloud2:
        def __init__(self):
            self.header = _Header()
            self.height = 0
            self.width = 0
            self.fields = []
            self.is_bigendian = False
            self.point_step = 0
            self.row_step = 0
            self.data = []
            self.is_dense = False

    class Imu:
        def __init__(self):
            self.header = _Header()
            self.orientation = None
            self.orientation_covariance = []
            self.angular_velocity = None
            self.angular_velocity_covariance = []
            self.linear_acceleration = None
            self.linear_acceleration_covariance = []

    class NavSatFix:
        def __init__(self):
            self.header = _Header()
            self.status = None
            self.latitude = 0.0
            self.longitude = 0.0
            self.altitude = 0.0
            self.position_covariance = []
            self.position_covariance_type = 0

    msg_mod.LaserScan = LaserScan
    msg_mod.PointCloud2 = PointCloud2
    msg_mod.Imu = Imu
    msg_mod.NavSatFix = NavSatFix


# ---------------------------------------------------------------------------
# 7. builtin_interfaces mocking
# ---------------------------------------------------------------------------

def _mock_builtin_interfaces():
    msg_mod = _make_module("builtin_interfaces", "msg")

    class Duration:
        def __init__(self, sec=0, nanosec=0):
            self.sec = sec
            self.nanosec = nanosec

    class Time:
        def __init__(self, sec=0, nanosec=0):
            self.sec = sec
            self.nanosec = nanosec

    msg_mod.Duration = Duration
    msg_mod.Time = Time


# ---------------------------------------------------------------------------
# 8. opennav_docking_msgs mocking
# ---------------------------------------------------------------------------

def _mock_opennav_docking_msgs():
    action_mod = _make_module("opennav_docking_msgs", "action")

    class _Duration:
        def __init__(self, sec=0, nanosec=0):
            self.sec = sec
            self.nanosec = nanosec

    # DockRobot
    class DockRobot:
        class Goal:
            def __init__(self):
                self.dock_id = ""
                self.dock_type = ""
                self.navigate_to_staging_pose = True

        class Result:
            def __init__(self):
                self.success = True
                self.num_retries = 0
                self.error_code = 0

        class Feedback:
            def __init__(self):
                self.state = 0
                self.docking_time = _Duration()
                self.num_retries = 0

    action_mod.DockRobot = DockRobot

    # UndockRobot
    class UndockRobot:
        class Goal:
            def __init__(self):
                self.dock_type = ""

        class Result:
            def __init__(self):
                self.success = True
                self.error_code = 0

        class Feedback:
            def __init__(self):
                self.state = 0
                self.undocking_time = _Duration()

    action_mod.UndockRobot = UndockRobot


# ---------------------------------------------------------------------------
# 9. Additional commonly-used ROS 2 packages
# ---------------------------------------------------------------------------

def _mock_action_msgs():
    """Stub action_msgs.msg for GoalStatus constants."""
    msg_mod = _make_module("action_msgs", "msg")

    class GoalStatus:
        STATUS_UNKNOWN = 0
        STATUS_ACCEPTED = 1
        STATUS_EXECUTING = 2
        STATUS_CANCELING = 3
        STATUS_SUCCEEDED = 4
        STATUS_CANCELED = 5
        STATUS_ABORTED = 6

    msg_mod.GoalStatus = GoalStatus


def _mock_lifecycle_msgs():
    """Stub lifecycle_msgs for nodes that import it."""
    msg_mod = _make_module("lifecycle_msgs", "msg")

    class State:
        PRIMARY_STATE_UNKNOWN = 0
        PRIMARY_STATE_UNCONFIGURED = 1
        PRIMARY_STATE_INACTIVE = 2
        PRIMARY_STATE_ACTIVE = 3
        PRIMARY_STATE_FINALIZED = 4

    msg_mod.State = State

    srv_mod = _make_module("lifecycle_msgs", "srv")

    class ChangeState:
        class Request:
            def __init__(self):
                self.transition = None

        class Response:
            def __init__(self):
                self.success = True

    srv_mod.ChangeState = ChangeState


def _mock_tf2_ros():
    """Stub tf2_ros for transform utilities."""
    mod = _make_module("tf2_ros")

    class Buffer:
        def __init__(self, *a, **kw):
            pass

        def lookup_transform(self, *a, **kw):
            return None

        def can_transform(self, *a, **kw):
            return True

    class TransformListener:
        def __init__(self, *a, **kw):
            pass

    class StaticTransformBroadcaster:
        def __init__(self, *a, **kw):
            pass

        def sendTransform(self, *a, **kw):
            pass

    class TransformBroadcaster:
        def __init__(self, *a, **kw):
            pass

        def sendTransform(self, *a, **kw):
            pass

    mod.Buffer = Buffer
    mod.TransformListener = TransformListener
    mod.StaticTransformBroadcaster = StaticTransformBroadcaster
    mod.TransformBroadcaster = TransformBroadcaster


def _mock_rcl_interfaces():
    """Stub rcl_interfaces for parameter events."""
    msg_mod = _make_module("rcl_interfaces", "msg")

    class ParameterEvent:
        def __init__(self):
            self.node = ""
            self.new_parameters = []
            self.changed_parameters = []
            self.deleted_parameters = []

    class SetParametersResult:
        def __init__(self):
            self.successful = True
            self.reason = ""

    msg_mod.ParameterEvent = ParameterEvent
    msg_mod.SetParametersResult = SetParametersResult


# ---------------------------------------------------------------------------
# Bootstrap: call all mock functions at import time
# ---------------------------------------------------------------------------

_mock_rclpy()
_mock_nav2_msgs()
_mock_geometry_msgs()
_mock_std_msgs()
_mock_nav_msgs()
_mock_sensor_msgs()
_mock_builtin_interfaces()
_mock_opennav_docking_msgs()
_mock_action_msgs()
_mock_lifecycle_msgs()
_mock_tf2_ros()
_mock_rcl_interfaces()
