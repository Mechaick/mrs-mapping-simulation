# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Darby Lim

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
import os.path


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    turtlebot3_cartographer_prefix = get_package_share_directory('mrs_mapping_simulation')
    cartographer_config_dir = LaunchConfiguration('cartographer_config_dir', default=os.path.join(
                                                  turtlebot3_cartographer_prefix, 'config'))
    #configuration_basename = LaunchConfiguration('configuration_basename',
                                                 #default='cartographer.lua')
    
    #Checking if the robot already have a config file, if not create it
    robot_name = os.environ.get('ROBOT_NAME')
    print("________________________Robot name: "+robot_name)
    custom_configuration_basename = LaunchConfiguration('configuration_basename',
                                                 default=robot_name+'.lua')
    

    cartographer_config_dir_str = get_package_share_directory('mrs_mapping_simulation')+ '/config'
    full_path = cartographer_config_dir_str+'/'+robot_name+'.lua'

    check_file = os.path.isfile(full_path)
    print("Checking if file exists: "+str(check_file)+" "+full_path)
    print("Creating new config file for robot "+robot_name)
    os.system("cp "+cartographer_config_dir_str+"/cartographer.lua "+full_path)
    os.system("sed -i 's/X/"+robot_name+"/g' "+full_path)

       


    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')



    return LaunchDescription([
        
        Node(
            package='mrs_mapping_simulation',
            executable='relative_tf',
            name='relative_tf',
            output='screen',
            node_namespace=robot_name,
            parameters=[{'use_sim_time': use_sim_time,'robot_name': robot_name}],
            # remappings = [("/joint_states","/tb3_0/joint_states"),
            #               ("/tf", "/tb3_0/tf"),
            #               ("/tf_static", "/tb3_0/tf_static")],
        ),
        
        DeclareLaunchArgument(
            'cartographer_config_dir',
            default_value=cartographer_config_dir,
            description='Full path to config file to load'),
        DeclareLaunchArgument(
            'configuration_basename',
            default_value=custom_configuration_basename,
            description='Name of lua file for cartographer'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            node_namespace=robot_name,
            parameters=[{'use_sim_time': use_sim_time}],
            remappings = [("/odom",robot_name+"/odom_relative"),("/scan",robot_name+"/scan"),],
            # remappings = [("/joint_states","/tb3_0/joint_states"),
            #               ("/tf", "/tb3_0/tf"),
            #               ("/tf_static", "/tb3_0/tf_static")],
            arguments=['-configuration_directory', cartographer_config_dir,
                       '-configuration_basename', custom_configuration_basename]),

        DeclareLaunchArgument(
            'resolution',
            default_value=resolution,
            description='Resolution of a grid cell in the published occupancy grid'),

        DeclareLaunchArgument(
            'publish_period_sec',
            default_value=publish_period_sec,
            description='OccupancyGrid publishing period'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/occupancy_grid.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time, 'resolution': resolution,
                              'publish_period_sec': publish_period_sec, 'robot_name':robot_name}.items(),
        ),

        
    ])
