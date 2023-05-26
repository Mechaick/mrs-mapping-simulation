# Requirements:
#   A realsense D400 series
#   Install realsense2 ros2 package (refactor branch)
# Example:
#   $ ros2 launch realsense2_camera rs_launch.py align_depth:=true
#
#   $ ros2 launch rtabmap_ros realsense_d400.launch.py
#   OR
#   $ ros2 launch rtabmap_ros rtabmap.launch.py frame_id:=camera_link args:="-d" rgb_topic:=/camera/color/image_raw depth_topic:=/camera/aligned_depth_to_color/image_raw camera_info_topic:=/camera/color/camera_info approx_sync:=false
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, EmitEvent, ExecuteProcess
from launch.substitutions import LaunchConfiguration,PythonExpression
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch.event_handlers import OnProcessStart
#from launch.events import 


def launch_setup(context, *args, **kwargs):
    robot_namespace = LaunchConfiguration('robot_namespace').perform(context)
    print(f'robot_namespace:  {robot_namespace}')
    parameters=[{
        #'frame_id':f'{robot_namespace}/base_footprint',
          'frame_id':f'base_link',
        #  'odom_frame_id':f'{robot_namespace}/odom',
          'odom_frame_id':f'odom',
        #  'map_frame_id':f'{robot_namespace}/map',
          'map_frame_id':f'map',
          'subscribe_depth':True,
          'approx_sync':True,
          'wait_for_transform': 0.99,
          # 'imu_topic':'rtabmap/imu',
          # 'wait_imu_to_init':False,
          # 'odom_frame_id': 'odom'
          }]
    remappings=[
          ('rgb/image', f'/{robot_namespace}/rgb_cam/image_raw'),
          ('rgb/camera_info', f'/{robot_namespace}/rgb_cam/camera_info'),
          ('depth/image', f'/{robot_namespace}/rgb_cam/depth/image_raw'),
          ('scan', f'/{robot_namespace}/scan'),
          ('odom', f'/{robot_namespace}/odom'),
          ('/tf', f'/{robot_namespace}/tf'),
          ('/tf_static', f'/{robot_namespace}/tf_static'),
          ]
    
    odometry_node =Node(
            package='rtabmap_ros', executable='rgbd_odometry', output='screen',
            parameters=parameters,
            node_namespace=robot_namespace,
            remappings=remappings)

    rtabmap_node  = Node(
            package='rtabmap_ros', executable='rtabmap', output='screen',
            parameters=parameters,
            node_namespace=robot_namespace,
            remappings=remappings,
            arguments=['-d'])
    
    return [odometry_node, rtabmap_node]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_namespace',default_value='box_bot0',description='Namespace of the robot'),
        OpaqueFunction(function=launch_setup)
    ])