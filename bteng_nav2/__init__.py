"""bteng_nav2 -- Nav2 behavior tree nodes for the BTEng engine."""

from importlib.metadata import PackageNotFoundError, version

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
from bteng_nav2.xml_integration import (
    BTENG_NODES,
    load_tree,
    port_model,
    register_nodes,
    registered_tags,
)

try:
    __version__ = version("bteng-nav2")
except PackageNotFoundError:
    __version__ = "unknown"

__all__ = [
    # Actions
    "NavigateToPoseNode",
    "NavigateThroughPosesNode",
    "FollowPathNode",
    "ComputePathToPoseNode",
    "ComputePathThroughPosesNode",
    "SmoothPathNode",
    # Recovery actions
    "SpinNode",
    "BackUpNode",
    "DriveOnHeadingNode",
    "WaitNode",
    "AssistedTeleopNode",
    # Services
    "ClearGlobalCostmapNode",
    "ClearLocalCostmapNode",
    "ClearGlobalCostmapAroundRobotNode",
    "ClearLocalCostmapAroundRobotNode",
    "GetGlobalCostmapNode",
    "GetLocalCostmapNode",
    "IsPathValidNode",
    "LoadMapNode",
    "SaveMapNode",
    "ManageLifecycleNodesNode",
    "SaveSlamMapNode",
    "SerializePoseGraphNode",
    "SetCostmapParameterNode",
    "GetCostmapParameterNode",
    # Conditions
    "IsRobotNearGoalCondition",
    "IsLocalizationActiveCondition",
    "IsObstacleNearCondition",
    "IsStuckCondition",
    "IsGoalUpdatedCondition",
    "IsNavigationActiveCondition",
    "IsBatteryLowCondition",
    "IsEmergencyStopCondition",
    "IsRobotAtFrameCondition",
    # Docking actions
    "DockRobotNode",
    "UndockRobotNode",
    # Publishers
    "PublishGoalPoseNode",
    "SetInitialPoseNode",
    # BTEng XML/factory integration
    "BTENG_NODES",
    "register_nodes",
    "load_tree",
    "registered_tags",
    "port_model",
    # Version
    "__version__",
]
