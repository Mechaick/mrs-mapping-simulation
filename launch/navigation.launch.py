# Copyright (c) 2018 Intel Corporation
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

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable,GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from nav2_common.launch import RewrittenYaml
from ruamel.yaml import YAML
import re
import yaml


def increase_ports(filename):
    with open(filename, 'r') as yaml_file:
        data = yaml.safe_load(yaml_file)

    i = int(re.search(r'\d+', filename).group())
    print("________________________Robot number: " + str(i))

    try:
        data['bt_navigator']['ros__parameters']['groot_zmq_publisher_port'] += i * 2
        data['bt_navigator']['ros__parameters']['groot_zmq_server_port'] += i * 2
    except KeyError:
        print("Error: Key not found in YAML file.")

    with open(filename, 'w') as yaml_file:
        yaml.safe_dump(data, yaml_file)


def create_new_config(pkg_mrs_mapping_simulation, robot_name):
    original_file_path = os.path.join(pkg_mrs_mapping_simulation, 'config', 'nav2_tb3_0.yaml')
    new_file_path = os.path.join(pkg_mrs_mapping_simulation, 'config', f'nav2_{robot_name}.yaml')

    # Read the original file as text
    with open(original_file_path, 'r') as f:
        file_content = f.read()

    # Replace all occurrences of 'tb3_0' with the new robot_name
    modified_content = file_content.replace('tb3_0', robot_name)

    # Write the modified content to a new file
    with open(new_file_path, 'w') as f:
        f.write(modified_content)


def generate_launch_description():
    # Get the launch directory
    try:
        robot_name = os.environ['ROBOT_NAME']
    except KeyError:
        robot_name = 'tb3_0'
        print("________________________Robot name EV not found, using default: "+robot_name)
    print("________________________Robot name: "+robot_name)

    bringup_dir = get_package_share_directory('nav2_bringup')
    
    pkg_mrs_mapping_simulation = get_package_share_directory('mrs_mapping_simulation')
    full_path = os.path.join(pkg_mrs_mapping_simulation, 'config/', 'nav2_'+robot_name+'.yaml')
    check_file = os.path.isfile(full_path)
    print("Checking if file exists: "+str(check_file)+" "+full_path)
    if(check_file == False):
        print("Creating new config file for robot "+robot_name)
        create_new_config(pkg_mrs_mapping_simulation, robot_name)

        increase_ports(full_path)

    default_params_file = os.path.join(get_package_share_directory("mrs_mapping_simulation"),
                                        'config', 'nav2_'+robot_name+'.yaml')
    print("Default params file: "+default_params_file)

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')
    map_subscribe_transient_local = LaunchConfiguration('map_subscribe_transient_local')

    lifecycle_nodes = ['controller_server',
                       'planner_server',
                       'recoveries_server',
                       'bt_navigator',
                       'waypoint_follower']

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    #remappings = [('/tf', 'tf'),
    #              ('/tf_static', 'tf_static')]
    remappings = []
    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'default_bt_xml_filename': default_bt_xml_filename,
        'autostart': autostart,
        'map_subscribe_transient_local': map_subscribe_transient_local}

    configured_params = RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True)


    controller_node = Node(
        package='nav2_controller',
        executable='controller_server',
        output='screen',
        parameters=[configured_params],
        remappings=remappings)

    planner_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[configured_params],
        remappings=remappings)

    recoveries_node = Node(
        package='nav2_recoveries',
        executable='recoveries_server',
        name='recoveries_server',
        output='screen',
        parameters=[configured_params],
        remappings=remappings)

    bt_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[configured_params],
        remappings=remappings)

    waypoint_node = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[configured_params],
        remappings=remappings)

    lifecycle_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': autostart},
                    {'node_names': lifecycle_nodes}])


    ld = LaunchDescription()

    ld.add_action(SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'))
    ld.add_action(DeclareLaunchArgument(
            'namespace', default_value=robot_name,
            description='Top-level namespace'))
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='false',
            description='Use simulation (Gazebo) clock if true'))
    ld.add_action(DeclareLaunchArgument('autostart', default_value='true',
            description='Automatically startup the nav2 stack'))
    ld.add_action(DeclareLaunchArgument('params_file',
            default_value=default_params_file,
            description='Full path to the ROS2 parameters file to use'))
    ld.add_action(DeclareLaunchArgument('default_bt_xml_filename',
            default_value=os.path.join(
                get_package_share_directory('nav2_bt_navigator'),
                'behavior_trees', 'navigate_w_replanning_and_recovery.xml'),
            description='Full path to the behavior tree xml file to use'))
    ld.add_action(DeclareLaunchArgument('map_subscribe_transient_local', default_value='false',
            description='Whether to set the map subscriber QoS to transient local'))
    ld.add_action(GroupAction([
        PushRosNamespace(robot_name),
        controller_node,
        planner_node,
        recoveries_node,
        bt_node,
        waypoint_node,
        lifecycle_node,
    ]))
    
    return ld