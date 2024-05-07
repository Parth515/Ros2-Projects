#!/usr/bin/python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_sam_gazebo_noise = get_package_share_directory('sam_description')
    pkg_sam_recognition = get_package_share_directory('sam_recognition')


    start_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_sam_gazebo_noise, 'launch', 'gazebo.launch.py'),
        )
    )
    
    start_world_noise = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_sam_gazebo_noise, 'launch', 'noise.launch.py'),
            )
        )

    spawn_yolo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_sam_recognition, 'launch', 'launch_yolov8.launch.py'),
        )
    )  

    return LaunchDescription([
        start_world,
        start_world_noise,
        spawn_yolo,
    ])