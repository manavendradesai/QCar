from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package = "lab1_cpp", 
            executable = "talker", 
            name = "talker_params",
            output = "screen",
            emulate_tty = True,
            parameters = [
                {"v":1.0, "d":1.0}
            ]
        ),
        Node(
            package = "lab1_cpp", 
            executable = "relay", 
            name = "relay",
            output = "screen",
            emulate_tty = True
        )
    ])

