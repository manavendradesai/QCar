from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        Node(
            package = "lab4", 
            
            # Python
            executable = "gap_follow.py",

            # # C++
            # executable = "gap_follow", 
                        parameters = [
                {"L_disp" : 1.5},
                {"L_look" : 1.0},
                {"deltaf_max" : 0.436}
            ],
        
            name = "gap_follow",
            output = "screen",
            emulate_tty = True
        )
        
    ])
