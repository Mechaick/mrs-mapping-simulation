#!/usr/bin/python3
# -*- coding: utf-8 -*-
import sys
import os
import random
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import TimerAction,OpaqueFunction,IncludeLaunchDescription, SetEnvironmentVariable, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution, Command
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir, LaunchConfiguration
import os.path

spawn_points = [[35.0, -3.0, 0.0], [28.0, -11.0, 0.0], [-25.0, 17.0, 0.0],[-27.0, 37.0, 0.0],[0.0, 0.0, 0.0],[-25.0, -6.0, 0.0],[-16.0, 33.0, 0.0],[1.0, -41.0, 0.0],[14.0, -30.0, 0.0],[34.0, -29.0, 0.0],[62.0, -23.0, 0.0],[74.0, -3.0, 0.0]]

def gen_robot_list(number_of_robots):

    robots = []
    print("Number of robots: "+str(number_of_robots))
    for i in range(int(number_of_robots)):
        robot_name = "tb3_"+str(i)
        x_pos = float(i)
        pkg_mrs_mapping_simulation = get_package_share_directory('mrs_mapping_simulation')

        full_path = os.path.join(pkg_mrs_mapping_simulation, 'robot/', 'turtlebot3_waffle'+str(i)+'.sdf')
        check_file = os.path.isfile(full_path)
        print("Checking if file exists: "+str(check_file)+" "+full_path)
        #if(check_file == False):
        print("Creating new urdf file for robot "+robot_name)
        os.system("cp "+pkg_mrs_mapping_simulation+"/robot/turtlebot3_waffleX.sdf "+full_path)
        os.system("sed -i 's/tb3_0/"+robot_name+"/g' "+full_path)
        spawn = spawn_points[random.randint(0,spawn_points.__len__()-1)]
        spawn_points.remove(spawn)
        robots.append({'name': robot_name, 'x_pose': spawn[0], 'y_pose':spawn[1], 'z_pose': spawn[2], 'path':full_path})


    return robots 

def launch_setup(context, *args, **kwargs):
    pkg_turtlebot_description = get_package_share_directory('turtlebot3_description')
    urdf = os.path.join(pkg_turtlebot_description, 'urdf/', 'turtlebot3_waffle.urdf')
    
    number_of_robots = LaunchConfiguration('number_of_robots').perform(context)
    
    #assert os.path.exists(urdf), "Thebox_bot.urdf doesnt exist in "+str(urdf)
    
    # Names and poses of the robots
    robots = gen_robot_list(number_of_robots)
    print("Number of robots: "+str(robots.__len__()))
    print("Type of robots: "+str(robots))
    print("Name of robots: "+str(robots[0]['name']))
    print("Type of number of robot: "+str(number_of_robots))
    

    # We create the list of spawn robots commands and the robot state publisher
    spawn_robots_cmds = []
    delayed_cmds=[]
    for robot in robots:
        print("Robot name: "+robot['name'])
        print(os.path.join(get_package_share_directory("mrs_mapping_simulation"),'config', 'slam_toolbox_'+robot['name']+'.yaml'))
        #spawn_robots_cmds.append(os.environ['ROBOT_NAME'] = robot['name'])
        spawn_robots_cmds.append(
            SetEnvironmentVariable(
                name="ROBOT_NAME",
                value=robot['name']
            )
        )
        spawn_robots_cmds.append(
            SetEnvironmentVariable(
                name="MAP_TOPIC",
                value="map"
            )
        )
        spawn_robots_cmds.append(
            
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[{'use_sim_time': True},
                        {'robot_description': ParameterValue(
                            Command(['xacro ', str(urdf)]), value_type=str),
                        'publish_frequency': 50.0, 'frame_prefix':str(robot['name']+"/")

                }],
                arguments=[urdf],
                
                node_namespace=robot['name'],
                #remappings = [("/joint_states",robot['name']+"/joint_states"),("/tf", robot['name'] + "/tf"),("/tf_static", robot['name'] + "/tf_static")],
            )
        )
        spawn_robots_cmds.append(
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                name='spawn_entity_' + robot['name'],
                output='screen',
                node_namespace=robot['name'],
                arguments=['-entity', robot['name'], '-x', str(robot['x_pose']), '-y', str(robot['y_pose']), '-z', str(robot['z_pose']), '-file', str(robot['path'])],
            )
         )
        # spawn_robots_cmds.append(
        #     Node(
        #         package='mrs_mapping_simulation',
        #         executable='relative_tf_tb',
        #         name='relative_tf_tb',
        #         output='screen',
        #         node_namespace=robot['name'],
        #         parameters=[{'use_sim_time': True,'robot_name': robot['name']}],
        #         # remappings = [("/joint_states","/tb3_0/joint_states"),
        #         #               ("/tf", "/tb3_0/tf"),
        #         #               ("/tf_static", "/tb3_0/tf_static")],
        #     ),
        # )
        
        spawn_robots_cmds.append(
            Node(
                package='mrs_mapping_simulation',
                executable='lidar_limiter',
                name='lidar_limiter' + robot['name'],
                output='screen',
                node_namespace=robot['name'],
                arguments=['robot_name', robot['name']],
            )
         )
        spawn_robots_cmds.append(
            Node(
                package='map_expand',
                executable='expander',
                name='expander' + robot['name'],
                output='screen',
                node_namespace=robot['name'],
            )
         )

        spawn_robots_cmds.append(
            IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/slam_toolbox.launch.py']),
            launch_arguments={'use_sim_time': 'true'}.items(),
        ),)
        # spawn_robots_cmds.append(
        #     IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/navigation.launch.py']),
        #     launch_arguments={'use_sim_time': 'true'}.items(),
        # ),)



    # Create the launch description and populate
    # ld = LaunchDescription()
    
    # for spawn_robot_cmd in spawn_robots_cmds:
    #     ld.add_action(spawn_robot_cmd)

    # ld.add_action(
    #         IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/slam_toolbox.launch.py']),
    #         launch_arguments={'use_sim_time': 'true','params_file':os.path.join(get_package_share_directory("mrs_mapping_simulation"),
    #                                    'config', 'slam_toolbox_tb3_1.yaml')}.items(),
    #     ),)

    return spawn_robots_cmds

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('number_of_robots', default_value='2'),
        OpaqueFunction(function=launch_setup)
    ])
    