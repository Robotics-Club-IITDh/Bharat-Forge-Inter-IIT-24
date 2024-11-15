import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
import launch
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

ARGUMENTS = [
    DeclareLaunchArgument('rviz', default_value='false',
                          choices=['true', 'false'],
                          description='Start rviz.'),
    DeclareLaunchArgument('world', default_value='empty_world',
                          description='Gazebo World'),
    DeclareLaunchArgument('robot_name', default_value='zinger',
                          description='Robot name'),
    DeclareLaunchArgument('x', default_value='0',
                          description='The x-coordinate for the robot'),
    DeclareLaunchArgument('y', default_value='0',
                          description='The y-coordinate for the robot'),
    DeclareLaunchArgument('z', default_value='0',
                          description='The z-coordinate for the robot'),
    DeclareLaunchArgument('yaw', default_value='0',
                          description='The rotation for the robot')
]

def generate_launch_description():
    # Directories
    pkg_robot_description = get_package_share_directory(
        'zinger_description')
    pkg_robot_control = get_package_share_directory(
        'zinger_swerve_controller'
    )
    pkg_robot_viz = get_package_share_directory(
        'zinger_viz')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    x, y, z = LaunchConfiguration('x'), LaunchConfiguration('y'), LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')

    # Gazebo
    gazebo_launch = PathJoinSubstitution([pkg_gazebo_ros, 'launch', 'gazebo.launch.py'])
    gazebo = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gazebo_launch]),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'pause': 'false',
            'use_sim_time': 'true',
            'gui': 'true',
            'debug': 'false',
            'verbose': 'false',
            'extra_gazebo_args': ''
        }.items()
    )

    # Spawn the robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', LaunchConfiguration('robot_name'),
            '-file', PathJoinSubstitution([
                pkg_robot_description, 'urdf', 'base.urdf']),
            '-x', x, '-y', y, '-z', z, '-Y', yaw
        ],
        output='screen'
    )

    # Robot description
    robot_description_base_launch = PathJoinSubstitution(
        [pkg_robot_description, 'launch', 'base.launch.py'])
    robot_description_base = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([robot_description_base_launch]),
        launch_arguments=[('use_sim_time', 'true')]
    )

    # Robot controllers
    robot_description_controller_launch = PathJoinSubstitution(
        [pkg_robot_description, 'launch', 'controllers.launch.py'])
    robot_controllers = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([robot_description_controller_launch]),
        launch_arguments=[
            ('use_sim_time', 'true'),
            ('use_fake_hardware', 'false'),
            ('fake_sensor_commands', 'false'),
        ]
    )

    # Rviz2
    rviz_launch = PathJoinSubstitution(
        [pkg_robot_viz, 'launch', 'view_robot.launch.py'])
    rviz2 = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([rviz_launch]),
        condition=IfCondition(LaunchConfiguration('rviz')),
        launch_arguments=[
            ('use_sim_time', 'true'),
            ('description', 'false')
        ]
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(gazebo)
    ld.add_action(spawn_entity)
    ld.add_action(robot_description_base)
    ld.add_action(robot_controllers)
    ld.add_action(rviz2)
    return ld
