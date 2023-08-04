import os


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, GroupAction
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import HasNodeParams



def generate_launch_description():
    try:
        robot_name = os.environ['ROBOT_NAME']
    except KeyError:
        robot_name = 'default'
        print("________________________Robot name EV not found, using default: "+robot_name)
    print("________________________Robot name: "+robot_name)

    #Checking if the config file is here, if not creating a new one (currently always creating to be ure to propagate changes)
    
    pkg_mrs_mapping_simulation = get_package_share_directory('mrs_mapping_simulation')
    full_path = os.path.join(pkg_mrs_mapping_simulation, 'config/', 'slam_toolbox_'+robot_name+'.yaml')
    check_file = os.path.isfile(full_path)
    print("Checking if file exists: "+str(check_file)+" "+full_path)
    #if(check_file == False):
    print("Creating new config file for robot "+robot_name)
    os.system("cp "+pkg_mrs_mapping_simulation+"/config/slam_toolbox_X.yaml "+full_path)
    os.system("sed -i 's/tb3_0/"+robot_name+"/g' "+full_path)




    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    default_params_file = os.path.join(get_package_share_directory("mrs_mapping_simulation"),
                                       'config', 'slam_toolbox_'+robot_name+'.yaml')

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation/Gazebo clock')
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node')

    # If the provided param file doesn't have slam_toolbox params, we must pass the
    # default_params_file instead. This could happen due to automatic propagation of
    # LaunchArguments. See:
    # https://github.com/ros-planning/navigation2/pull/2243#issuecomment-800479866
    
    
    has_node_params = HasNodeParams(source_file=params_file,
                                    node_name=robot_name+'/slam_toolbox')



    actual_params_file = PythonExpression(['"', params_file, '" if ', has_node_params,
                                           ' else "', default_params_file, '"'])

    log_param_change = LogInfo(msg=['provided params_file ',  params_file,
                                    ' does not contain slam_toolbox parameters. Using default: ',
                                    default_params_file],
                               condition=UnlessCondition(has_node_params))

    start_async_slam_toolbox_node = Node(
        #namespace=robot_name,
        parameters=[
          actual_params_file,
          {'use_sim_time': use_sim_time},
        ],
        remappings=[
                    ('/map',  'map'),('/map_metadata',  'map_metadata'),
                    ('/slam_toolbox/feedback','slam_toolbox/feedback'),
                    ('/slam_toolbox/graph_visualization', 'slam_toolbox/graph_visualization'),
                    ('/slam_toolbox/scan_visualization', 'slam_toolbox/scan_visualization')],
        package='slam_toolbox_open',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen')

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(log_param_change)
    ld.add_action(GroupAction([
        PushRosNamespace(robot_name),
        start_async_slam_toolbox_node,
    ]))

    return ld
