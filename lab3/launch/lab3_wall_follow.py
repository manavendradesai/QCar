from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        Node(
            package = "lab3", 
            
            # # Python
            # executable = "wall_follow.py",

            # # C++
            executable = "wall_follow", 
                        parameters = [
                {"TTC_cr" : 1.5},
                {"rnghead_low" : 0.524},
                {"rnghead_low" : 1.571},
                {"L" : 1.5},
                {"kp" : 0.5},
                {"kd" : 1.0},
                {"dist_to_wall" : 0.8}
            ],
        
            name = "wall_follow",
            output = "screen",
            emulate_tty = True
        )
    ])
