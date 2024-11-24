import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess,\
                           IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

import xacro

def generate_launch_description():
    pkg_name = 'axebot_description'
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')

    # Launch arguments
    launch_rviz = LaunchConfiguration('launch_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Declare launch arguments
    launch_rviz_arg = DeclareLaunchArgument(
        name='launch_rviz',
        default_value='False',
        description='True if to launch rviz, false otherwise'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    # File paths
    xacro_file = os.path.join(
        get_package_share_directory(pkg_name),
        'urdf',
        'axebot.urdf.xacro'
    )

    rviz_config = os.path.join(
      get_package_share_directory(pkg_name),
      'config',
      'axebot.rviz'
    )

    # Process XACRO
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros_dir, 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'world': ''
            }.items()
        )

    # Nodes
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_raw,
            'use_sim_time': use_sim_time
        }]
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                  '-entity', 'axebot'],
        output='screen'
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"]
    )

    omni_base_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["omnidirectional_controller"]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        condition=IfCondition(launch_rviz),
        arguments=['-d', rviz_config]
    )

    # Add fixed timer of 5 seconds before launching any nodes
    timer = TimerAction(
        period=5.0,
        actions=[
            robot_state_publisher_node,
            spawn_entity,
            joint_state_broadcaster_spawner,
            omni_base_controller_spawner,
            rviz_node
        ]
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,  # Add the declaration to LaunchDescription
        launch_rviz_arg,
        gazebo,
        timer
    ])
