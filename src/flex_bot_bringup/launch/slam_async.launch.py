import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share = get_package_share_directory("flex_bot_bringup")
    default_params = os.path.join(pkg_share, "config", "slam_config.yaml")
    slam_params_file = LaunchConfiguration("slam_params_file")

    slam_share = get_package_share_directory("slam_toolbox")
    slam_launch = os.path.join(slam_share, "launch", "online_async_launch.py")

    return LaunchDescription([
        DeclareLaunchArgument("slam_params_file", default_value=default_params),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch),
            launch_arguments={
                "slam_params_file": slam_params_file
            }.items(),
        ),
    ])
