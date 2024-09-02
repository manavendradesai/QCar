from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        Node(
            package = "uav_lenviz", 

            ## C++
            executable = "drone_operation",
            namespace = "uav_3",
            output = "screen",
            emulate_tty = True
        ),

        Node(
            package = "uav_lenviz", 
        
            ## C++
            executable = "mission_plan",
            parameters = [
                {"start_x" : 1},
                {"start_y" : 15},
                {"goal_x" : 10},
                {"goal_y" : 30},
                {"drone_id" : 3}
            ],
            namespace = "uav_3",
            output = "screen",
            emulate_tty = True
        ),

        Node(
            package = "uav_lenviz", 
        
            ## C++
            executable = "survey",
            parameters = [
                {"v_max" : 15.0},
                {"drone_id" : 3}
            ],
            namespace = "uav_3",
            output = "screen",
            emulate_tty = True
        )
    ])
