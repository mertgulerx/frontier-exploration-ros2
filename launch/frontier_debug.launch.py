from pathlib import Path
from typing import Any

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _load_shared_params(params_path: str) -> dict[str, Any]:
    path = Path(params_path).expanduser()
    if not path.exists():
        return {}

    with path.open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream) or {}

    # Parameters are merged in the same order ROS users normally expect:
    # wildcard defaults, explorer-specific values, then debug-specific overrides.
    # This lets the observer mirror the explorer while still allowing RViz-only
    # debug settings to live in the same YAML file.
    shared = {}
    wildcard_params = data.get("/**", {}).get("ros__parameters", {})
    if isinstance(wildcard_params, dict):
        shared.update(wildcard_params)

    frontier_params = data.get("frontier_explorer", {}).get("ros__parameters", {})
    if isinstance(frontier_params, dict):
        shared.update(frontier_params)

    debug_params = data.get("frontier_debug_observer", {}).get("ros__parameters", {})
    if isinstance(debug_params, dict):
        shared.update(debug_params)

    return shared


def _create_debug_actions(context):
    namespace = LaunchConfiguration("namespace")
    params_file = LaunchConfiguration("params_file").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time")
    log_level = LaunchConfiguration("log_level")

    debug_params = _load_shared_params(params_file)
    # Launch arguments override the YAML for fast RViz tuning. These parameters
    # affect only the observer's display/update behavior, not exploration logic.
    debug_params.update(
        {
            "use_sim_time": use_sim_time,
            "debug_update_rate_hz": LaunchConfiguration("debug_update_rate_hz"),
            "debug_labels_enabled": LaunchConfiguration("debug_labels_enabled"),
            "debug_label_top_n": LaunchConfiguration("debug_label_top_n"),
            "debug_edge_top_n": LaunchConfiguration("debug_edge_top_n"),
        }
    )

    return [
        Node(
            package="frontier_exploration_ros2",
            executable="frontier_debug_observer",
            name="frontier_debug_observer",
            namespace=namespace,
            output="screen",
            arguments=["--ros-args", "--log-level", log_level],
            parameters=[debug_params],
        )
    ]


def generate_launch_description():
    default_params = Path(
        get_package_share_directory("frontier_exploration_ros2")
    ) / "config" / "params.yaml"

    # The debug observer runs as a standalone node. It can be launched next to an
    # explorer instance or by itself against recorded map/costmap/TF data.
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "namespace",
                default_value="",
                description="Optional namespace for the frontier debug observer.",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=str(default_params),
                description="Parameter file used to mirror frontier_explorer settings.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation time.",
            ),
            DeclareLaunchArgument(
                "log_level",
                default_value="info",
                description="Log level (debug, info, warn, error, fatal).",
            ),
            DeclareLaunchArgument(
                "debug_update_rate_hz",
                default_value="1.0",
                description="Debug overlay update frequency.",
            ),
            DeclareLaunchArgument(
                "debug_labels_enabled",
                default_value="true",
                description="Publish text labels for top-ranked debug candidates.",
            ),
            DeclareLaunchArgument(
                "debug_label_top_n",
                default_value="30",
                description="Maximum number of candidate labels per overlay.",
            ),
            DeclareLaunchArgument(
                "debug_edge_top_n",
                default_value="15",
                description="Maximum number of MRTSP start edges drawn in RViz.",
            ),
            OpaqueFunction(function=_create_debug_actions),
        ]
    )
