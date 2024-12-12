import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch.actions import TimerAction

def generate_launch_description():
    pkg_dir = get_package_share_directory('autobots')
    xacro_file = os.path.join(pkg_dir, 'urdf', 'moving_bot.urdf.xacro')

    # Robot configurations
    robot1_spawn_x = 7.782
    robot1_spawn_y = 0.042
    robot1_spawn_z = 0.0

    robot2_spawn_x = 5.501
    robot2_spawn_y = -5.816
    robot2_spawn_z = 0.0

    robot3_spawn_x = 25.860
    robot3_spawn_y = -14.543
    robot3_spawn_z = 0.0

    # Robot 1 nodes
    robot1_description = Command(['xacro ', xacro_file])
    robot1_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot1_state_publisher',
        parameters=[{'robot_description': robot1_description}],
        remappings=[('/robot_description', '/robot_1/robot_description')],
        namespace='robot_1'
    )

    robot1_spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_robot1',
        output='screen',
        arguments=[
            '-topic', '/robot_1/robot_description',
            '-entity', 'robot_1',
            '-x', str(robot1_spawn_x),
            '-y', str(robot1_spawn_y),
            '-z', str(robot1_spawn_z)
        ]
    )

    robot1_controller = Node(
        package='autobots',
        executable='waypoint_follower',
        name='waypoint_follower_robot1',
        output='screen'
    )

    # Robot 2 nodes
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

    # Robot 3 nodes
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

    # Create launch description with delayed spawns
    ld = LaunchDescription([
        # Robot 1 - State publisher and spawn
        robot1_state_publisher,
        robot1_spawn,
        
        # Robot 1 controller - Start after 5 seconds
        TimerAction(
            period=5.0,
            actions=[robot1_controller]
        ),
        
        # Robot 2 - Launch after 15 seconds
        TimerAction(
            period=15.0,
            actions=[
                robot2_state_publisher,
                robot2_spawn,
            ]
        ),
        
        # Robot 2 controller - Start after 20 seconds
        TimerAction(
            period=20.0,
            actions=[robot2_controller]
        ),
        
        # Robot 3 - Launch after 30 seconds
        TimerAction(
            period=30.0,
            actions=[
                robot3_state_publisher,
                robot3_spawn,
            ]
        ),
        
        # Robot 3 controller - Start after 35 seconds
        TimerAction(
            period=35.0,
            actions=[robot3_controller]
        ),
    ])

    return ld