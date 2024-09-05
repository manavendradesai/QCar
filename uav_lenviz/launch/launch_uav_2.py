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
                {"start_x" : 100.0},
                {"start_y" : 5.0},
                {"goal_x" : 20.0},
                {"goal_y" : 5.0},
                {"drone_id" : 2},
                {"drone_num" : 2},
                {"crit_gap" : 2.0}
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
                {"drone_id" : 2},
                {"v" : 5.0},
                {"dt" : 0.5}
            ],
            namespace = "uav_2",
            output = "screen",
            emulate_tty = True
        )
    ])
