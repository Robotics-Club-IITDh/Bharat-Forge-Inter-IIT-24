from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from typing import List

def generate_launch_description():
    # Declare launch arguments
    declare_robot_names = DeclareLaunchArgument(
        'robot_names',
        default_value='["robot_1", "robot_2"]',  # Default value as a string representation of a list
        description='List of robot names to track'
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time if true'
    )

    # Get launch configurations
    robot_names = LaunchConfiguration('robot_names')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Position publisher node
    position_publisher_node = Node(
        package='robot_gazebo',
        executable='position_publisher',
        name='position_publisher',
        parameters=[{
            'robot_names': robot_names,
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )

    return LaunchDescription([
        declare_robot_names,
        declare_use_sim_time,
        position_publisher_node
    ])