from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():

    # Default world
    pkg_dir = get_package_share_directory('robot_gazebo')
    default_world = os.path.join(pkg_dir, 'world', 'empty.world')

    # Declare the 'world' launch argument with a default value
    world_arg = DeclareLaunchArgument('world',default_value= default_world)

    # Get the value of the 'world' argument
    world_file = LaunchConfiguration('world')


    # Gazebo directory
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')

    # Define the Gazebo process
    gazebo_process = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros_dir, 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={'world': world_file}.items()
        )

    # Return the launch description
    return LaunchDescription([
        world_arg,
        gazebo_process
    ])
