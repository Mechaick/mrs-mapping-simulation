#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, EmitEvent, ExecuteProcess,LogInfo, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import (OnExecutionComplete, OnProcessExit,
                                OnProcessIO, OnProcessStart, OnShutdown)
from launch.substitutions import LaunchConfiguration, FindExecutable
from launch_ros.actions import Node

def generate_launch_description():
    pkg_box_bot_description = get_package_share_directory('box_bot_description')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_mrs_mapping_simulation = get_package_share_directory('mrs_mapping_simulation')

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
        )
    )
    # Spawn robot launch
    spawn_robot_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_mrs_mapping_simulation, 'launch', 'multi_robot_spawn_rgbd.launch.py'),
        )
    )     

    # Create the launch description and populate
    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value="~/ros2_ws/src/mrs_mapping_simulation/worlds/moon.world",
            description='SDF world file'),
        DeclareLaunchArgument(
            'number_of_robots',
            default_value='2',
            description='Number of robots to spawn'),
        gazebo,
        TimerAction(
            period=5.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(pkg_mrs_mapping_simulation, 'launch', 'multi_robot_spawn_rgbd.launch.py'),
                    ),
                    launch_arguments={'number_of_robots': LaunchConfiguration('number_of_robots')}.items()
                )
            ],
        )
        
    ])

