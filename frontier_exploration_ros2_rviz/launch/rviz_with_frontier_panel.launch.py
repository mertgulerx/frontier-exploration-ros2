from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("frontier_exploration_ros2_rviz"))
    default_rviz_config = package_share / "rviz" / "frontier_exploration_panel.rviz"

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "rviz_config",
                default_value=str(default_rviz_config),
                description="RViz config that preloads the frontier exploration control panel.",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2_frontier_panel",
                output="screen",
                arguments=["-d", LaunchConfiguration("rviz_config")],
            ),
        ]
    )
