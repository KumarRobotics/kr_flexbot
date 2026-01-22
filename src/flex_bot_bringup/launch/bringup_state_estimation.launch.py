import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("flex_bot_bringup")
    default_ekf = os.path.join(pkg_share, "config", "ekf_imu.yaml")
    ekf_yaml = LaunchConfiguration("ekf_yaml")

    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[ekf_yaml],
    )

    return LaunchDescription([
        DeclareLaunchArgument("ekf_yaml", default_value=default_ekf),
        ekf_node,
    ])
