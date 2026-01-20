import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("li_bot_bringup")
    default_yaml = os.path.join(pkg_share, "config", "slam_config.yaml")
    slam_yaml = LaunchConfiguration("slam_yaml")

    slam_node = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[slam_yaml],
    )

    return LaunchDescription([
        DeclareLaunchArgument("slam_yaml", default_value=default_yaml),
        slam_node,
    ])
