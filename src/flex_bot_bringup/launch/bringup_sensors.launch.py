from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    hostname = LaunchConfiguration("hostname")
    udp_receiver_ip = LaunchConfiguration("udp_receiver_ip")
    publish_frame_id = LaunchConfiguration("publish_frame_id")
    host_FREchoFilter = LaunchConfiguration("host_FREchoFilter")
    tf_publish_rate = LaunchConfiguration("tf_publish_rate")

    sick_cmd = [
        "ros2", "launch", "sick_scan_xd", "sick_picoscan.launch.py",
        PythonExpression(["'hostname:=' + '", hostname, "'"]),
        PythonExpression(["'udp_receiver_ip:=' + '", udp_receiver_ip, "'"]),
        PythonExpression(["'publish_frame_id:=' + '", publish_frame_id, "'"]),
        PythonExpression(["'host_FREchoFilter:=' + '", host_FREchoFilter, "'"]),
        PythonExpression(["'tf_publish_rate:=' + '", tf_publish_rate, "'"]),
    ]

    return LaunchDescription([
        DeclareLaunchArgument("hostname", default_value="192.168.0.1"),
        DeclareLaunchArgument("udp_receiver_ip", default_value="192.168.0.20"),
        DeclareLaunchArgument("publish_frame_id", default_value="laser"),
        DeclareLaunchArgument("host_FREchoFilter", default_value="0"),
        DeclareLaunchArgument("tf_publish_rate", default_value="0"),

        ExecuteProcess(cmd=sick_cmd, output="screen"),
    ])
