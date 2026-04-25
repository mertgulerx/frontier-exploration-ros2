from pathlib import Path
from typing import Any

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _create_frontier_actions(context):
    namespace = LaunchConfiguration("namespace")
    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    log_level = LaunchConfiguration("log_level")
    autostart_value = LaunchConfiguration("autostart").perform(context)
    control_service_enabled_value = LaunchConfiguration("control_service_enabled").perform(context)
    map_qos_durability = LaunchConfiguration("map_qos_durability")
    map_qos_autodetect_on_startup = LaunchConfiguration("map_qos_autodetect_on_startup")
    map_qos_autodetect_timeout_s = LaunchConfiguration("map_qos_autodetect_timeout_s")
    costmap_qos_reliability = LaunchConfiguration("costmap_qos_reliability")

    frontier_overrides: dict[str, Any] = {
        "use_sim_time": use_sim_time,
        "map_qos_durability": map_qos_durability,
        "map_qos_autodetect_on_startup": map_qos_autodetect_on_startup,
        "map_qos_autodetect_timeout_s": map_qos_autodetect_timeout_s,
        "costmap_qos_reliability": costmap_qos_reliability,
    }
    if autostart_value != "":
        frontier_overrides["autostart"] = autostart_value
    if control_service_enabled_value != "":
        frontier_overrides["control_service_enabled"] = control_service_enabled_value

    frontier_node = Node(
        package="frontier_exploration_ros2",
        executable="frontier_explorer",
        name="frontier_explorer",
        namespace=namespace,
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[
            params_file,
            frontier_overrides,
        ],
    )

    return [frontier_node]


def generate_launch_description():
    # Project-specific defaults are kept in the repository root config folder.
    # This path is intentionally relative to the project root.
    default_params = Path("config/frontier_exploration_ros2/config.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "namespace",
                default_value="",
                description="Optional namespace for the frontier explorer node.",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=str(default_params),
                description="Parameter file for frontier_explorer.",
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
                "autostart",
                default_value="",
                description="Optional override for explorer autostart. Leave empty to use the parameter file.",
            ),
            DeclareLaunchArgument(
                "control_service_enabled",
                default_value="",
                description=(
                    "Optional override for frontier control service availability. "
                    "Leave empty to use the parameter file."
                ),
            ),
            DeclareLaunchArgument(
                "map_qos_durability",
                default_value="transient_local",
                description="Map durability: transient_local | volatile | system_default.",
            ),
            DeclareLaunchArgument(
                "map_qos_autodetect_on_startup",
                default_value="false",
                description="Enable startup-only map durability autodetect.",
            ),
            DeclareLaunchArgument(
                "map_qos_autodetect_timeout_s",
                default_value="2.0",
                description="Autodetect timeout per attempt in seconds.",
            ),
            DeclareLaunchArgument(
                "costmap_qos_reliability",
                default_value="reliable",
                description="Costmap reliability: reliable | best_effort | system_default.",
            ),
            # Packaged params are intentionally generic and expected to be adapted
            # for robot-specific frames, topics, and planner/controller behavior.
            OpaqueFunction(function=_create_frontier_actions),
        ]
    )
