from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        Node(
            package = "lab3", 
            # Python
            executable = "wall_follow.py",
            # # C++
            # executable = "wall_follow", 
            name = "wall_follow",
            output = "screen",
            emulate_tty = True
        )
    ])
