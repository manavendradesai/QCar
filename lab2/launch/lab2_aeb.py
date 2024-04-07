from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package = "lab2", 
            executable = "aeb", 
            name = "aeb",
            output = "screen",
            emulate_tty = True,
            parameters = [
                {"TTC_cr" : 1.5}
            ]
        )
    ])

