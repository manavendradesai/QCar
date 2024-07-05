from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     arguments = ['-0.11', '0', '0', '0', '0', '0', 'ego_racecar/laser_model', 'car_center']
        # ),
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

