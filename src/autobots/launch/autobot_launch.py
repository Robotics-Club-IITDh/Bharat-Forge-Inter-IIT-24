import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command

def generate_launch_description():
    # Get paths and files
    pkg_dir = get_package_share_directory('autobots')
    xacro_file = os.path.join(pkg_dir, 'urdf', 'moving_bot.urdf.xacro')

    # Set the spawn position to match first waypoint
    spawn_x = 7.782
    spawn_y = 0.042
    spawn_z = 0.0

    robot_description_config = Command(['xacro ', xacro_file])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description_config}],
        output='screen'
    )

    spawn_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_robot',
        output='screen',
        arguments=[
            '-topic', '/robot_description',
            '-entity', 'autobot',
            '-x', str(spawn_x),
            '-y', str(spawn_y),
            '-z', str(spawn_z)
        ]
    )

    waypoint_follower = Node(
        package='autobots',
        executable='waypoint_follower',
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher,
        spawn_node,
        waypoint_follower
    ])