import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command

def generate_launch_description():
    pkg_dir = get_package_share_directory('autobots')
    xacro_file = os.path.join(pkg_dir, 'urdf', 'moving_bot.urdf.xacro')

    # Robot 2 configuration
    robot2_spawn_x = 5.501
    robot2_spawn_y = -5.816
    robot2_spawn_z = 0.0

    robot2_description = Command(['xacro ', xacro_file])
    robot2_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot2_state_publisher',
        parameters=[{'robot_description': robot2_description}],
        remappings=[('/robot_description', '/robot_2/robot_description')],
        namespace='robot_2'
    )

    robot2_spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_robot2',
        output='screen',
        arguments=[
            '-topic', '/robot_2/robot_description',
            '-entity', 'robot_2',
            '-x', str(robot2_spawn_x),
            '-y', str(robot2_spawn_y),
            '-z', str(robot2_spawn_z)
        ]
    )

    robot2_controller = Node(
        package='autobots',
        executable='waypoint_follower_robot2',
        name='waypoint_follower_robot2',
        output='screen'
    )

    return LaunchDescription([
        robot2_state_publisher,
        robot2_spawn,
        robot2_controller
    ])