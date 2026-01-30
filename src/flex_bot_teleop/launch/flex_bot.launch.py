from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="flex_bot_teleop",
            executable="flex_bot_udp_bridge",
            name="flex_bot_udp_bridge",
            output="screen",
            parameters=["config/flex_bot_udp.yaml"],
        ),
        # Optional: your joystick node
        Node(
            package="flex_bot_teleop",
            executable="teleop_node",
            name="teleop_node",
            output="screen",
            parameters=["config/teleop.yaml"],
        ),
    ])
