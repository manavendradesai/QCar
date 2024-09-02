import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    
    uav_1 = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([os.path.join(
        get_package_share_directory('uav_lenviz'), 'launch'),
        '/launch_uav_1.py'])
    )

    uav_2 = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([os.path.join(
        get_package_share_directory('uav_lenviz'), 'launch'),
        '/launch_uav_2.py'])
    )

    uav_3 = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([os.path.join(
        get_package_share_directory('uav_lenviz'), 'launch'),
        '/launch_uav_3.py'])
    )

    uav_4 = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([os.path.join(
        get_package_share_directory('uav_lenviz'), 'launch'),
        '/launch_uav_4.py'])
    )

    uav_5 = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([os.path.join(
        get_package_share_directory('uav_lenviz'), 'launch'),
        '/launch_uav_5.py'])
    )

    return LaunchDescription([
        uav_1,
        uav_2,
        uav_3,
        uav_4,
        uav_5
    ])