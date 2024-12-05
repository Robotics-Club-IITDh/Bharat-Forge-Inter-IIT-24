from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from launch_ros.actions import Node

def generate_launch_description():

    pkg_dir = get_package_share_directory('robot_gazebo')
    default_world = os.path.join(pkg_dir, 'world', 'empty.world')

    world_arg = DeclareLaunchArgument('world', default_value=default_world)
    world_file = LaunchConfiguration('world')

    gazebo_ros_dir = get_package_share_directory('gazebo_ros')

    gazebo_process = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_file}.items()
    )
    
    robot_names = ['robot_1', 'robot_2']    # CHANGE FOR ALL ROBOTS

    merger_node = Node(
        package='robot_gazebo',
        executable='map_merger',
        name='map_merger',
        output='screen',
        parameters=[{
            'robot_namespaces': robot_names,
            'use_sim_time': True,
            'publish_rate': 10.0,
        }],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ]
    )

    master_controller_launch = Node(
        package='robot_gazebo', 
        executable='master_controller', 
        name='master_controller',
        output='screen',
        parameters=[{
            'robot_names': robot_names
        }],
        remappings=[('/target', '/target')] 
    )

    position_publisher_node = Node(
        package='robot_gazebo',
        executable='position_publisher',
        name='position_publisher',
        parameters=[{
            'robot_names': robot_names,
            'use_sim_time': True
        }],
        output='screen'
    )


    return LaunchDescription([
        world_arg,
        gazebo_process,
        merger_node,
        master_controller_launch,
        position_publisher_node
    ])
