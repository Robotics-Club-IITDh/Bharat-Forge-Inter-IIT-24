import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Path to the Xacro file
    pkg_dir = get_package_share_directory('autobots')
    xacro_file = os.path.join(pkg_dir, 'urdf', 'model.urdf.xacro')

    # Generate the robot description from the Xacro file
    xacro_command = ['xacro', xacro_file]
    try:
        robot_description = subprocess.check_output(xacro_command).decode('utf-8')
    except subprocess.CalledProcessError as e:
        raise RuntimeError(f"Failed to process Xacro file: {e}")

    # Spawn the robot in Gazebo at the origin
    spawn_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_robot',
        output='screen',
        arguments=[
            '-entity', 'my_robot',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.0',
            '-Y', '0.0',
            '-robot_namespace', '',
            '-param', '/robot_description',
        ],
        parameters=[{'/robot_description': robot_description}]
    )

    return LaunchDescription([spawn_node])
