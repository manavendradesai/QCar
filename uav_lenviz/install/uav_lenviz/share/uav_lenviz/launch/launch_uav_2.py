from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        Node(
            package = "uav_lenviz", 

            ## C++
            executable = "drone_operation",
            namespace = "uav_2",
            output = "screen",
            emulate_tty = True
        ),

        Node(
            package = "uav_lenviz", 
        
            ## C++
            executable = "mission_plan",
            parameters = [
                {"start_x" : 10},
                {"start_y" : 1},
                {"goal_x" : 20},
                {"goal_y" : 30},
                {"drone_id" : 2}
            ],
            namespace = "uav_2",
            output = "screen",
            emulate_tty = True
        ),

        Node(
            package = "uav_lenviz", 
        
            ## C++
            executable = "survey",
            parameters = [
                {"v_max" : 15.0},
                {"drone_id" : 2}
            ],
            namespace = "uav_2",
            output = "screen",
            emulate_tty = True
        )
    ])
