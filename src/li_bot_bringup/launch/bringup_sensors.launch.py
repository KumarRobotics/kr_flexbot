import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    hostname = LaunchConfiguration("hostname")
    udp_receiver_ip = LaunchConfiguration("udp_receiver_ip")
    publish_frame_id = LaunchConfiguration("publish_frame_id")
    host_FREchoFilter = LaunchConfiguration("host_FREchoFilter")
    tf_publish_rate = LaunchConfiguration("tf_publish_rate")

    sick_share = get_package_share_directory("sick_scan_xd")
    sick_launch = os.path.join(sick_share, "launch", "sick_picoscan.launch.py")

    return LaunchDescription([
        DeclareLaunchArgument("hostname", default_value="192.168.0.1"),
        DeclareLaunchArgument("udp_receiver_ip", default_value="192.168.0.20"),
        DeclareLaunchArgument("publish_frame_id", default_value="laser"),
        DeclareLaunchArgument("host_FREchoFilter", default_value="0"),
        # keep as float-string to avoid type mismatch
        DeclareLaunchArgument("tf_publish_rate", default_value="0.0"),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sick_launch),
            launch_arguments={
                "hostname": hostname,
                "udp_receiver_ip": udp_receiver_ip,
                "publish_frame_id": publish_frame_id,
                "host_FREchoFilter": host_FREchoFilter,
                "tf_publish_rate": tf_publish_rate,
            }.items(),
        ),
    ])
