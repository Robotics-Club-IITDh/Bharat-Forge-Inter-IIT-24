import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command

def generate_launch_description():
    pkg_dir = get_package_share_directory('autobots')
    xacro_file = os.path.join(pkg_dir, 'urdf', 'moving_bot.urdf.xacro')

    # Robot 3 configuration
    robot3_spawn_x = 25.860
    robot3_spawn_y = -14.543
    robot3_spawn_z = 0.0

    robot3_description = Command(['xacro ', xacro_file])
    robot3_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot3_state_publisher',
        parameters=[{'robot_description': robot3_description}],
        remappings=[('/robot_description', '/robot_3/robot_description')],
        namespace='robot_3'
    )

    robot3_spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_robot3',
        output='screen',
        arguments=[
            '-topic', '/robot_3/robot_description',
            '-entity', 'robot_3',
            '-x', str(robot3_spawn_x),
            '-y', str(robot3_spawn_y),
            '-z', str(robot3_spawn_z)
        ]
    )

    robot3_controller = Node(
        package='autobots',
        executable='waypoint_follower_robot3',
        name='waypoint_follower_robot3',
        output='screen'
    )

    return LaunchDescription([
        robot3_state_publisher,
        robot3_spawn,
        robot3_controller
    ])