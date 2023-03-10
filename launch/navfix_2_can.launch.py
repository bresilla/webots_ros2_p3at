from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    navfix2can = Node(
            package='webots_ros2_p3at',
            executable='navfix2can',
    )

    ld.add_action(navfix2can)
    return ld
