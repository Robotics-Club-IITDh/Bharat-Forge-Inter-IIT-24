from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node

ARGUMENTS = [
    DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'),
    DeclareLaunchArgument(
        "use_fake_hardware",
        default_value="true",
        description="Start robot with fake hardware mirroring command to its states.",
    ),
    DeclareLaunchArgument(
        "fake_sensor_commands",
        default_value="false",
        description="Enable fake command interfaces for sensors used for simple simulations. Used only if 'use_fake_hardware' parameter is true.",
    ),
    DeclareLaunchArgument(
        'description',
        default_value='true',
        description='Launch robot description'
    )
]


def generate_launch_description():

    pkg_robot_viz = get_package_share_directory('zinger_viz')
    pkg_robot_description = get_package_share_directory('zinger_description')

    rviz2_config = PathJoinSubstitution(
        [pkg_robot_viz, 'rviz', 'robot.rviz'])
    robot_description_launch = PathJoinSubstitution(
        [pkg_robot_description, 'launch', 'robot_description.launch.py']
    )

    robot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([robot_description_launch]),
        launch_arguments=[
            ('use_sim_time', LaunchConfiguration('use_sim_time')),
            ('use_fake_hardware', LaunchConfiguration('use_fake_hardware')),
            ('fake_sensor_commands', LaunchConfiguration('fake_sensor_commands'))
        ],
        condition=IfCondition(LaunchConfiguration('description'))
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz2_config],
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        output='screen')

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(robot_description)
    ld.add_action(rviz2)
    return ld