#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, GroupAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from launch.event_handlers import OnProcessExit
from launch.actions import IncludeLaunchDescription

def generate_launch_description():
    pkg_dir = get_package_share_directory('turtlebot3_multi_robot')
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')
    
    # Load the SLAM configuration file
    slam_config = os.path.join(pkg_dir, 'config', 'slam_config.yaml')
    
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_file = LaunchConfiguration(
        'world_file',
        default=os.path.join(pkg_dir, 'worlds', 'multi_empty_world.world')
    )

    # Robot configurations
    robots = [
        {'name': 'tb1', 'x': '0.0', 'y': '0.0', 'z': '0.01', 'yaw': '0.0'},
        {'name': 'tb2', 'x': '1.0', 'y': '0.0', 'z': '0.01', 'yaw': '0.0'}
        {'name': 'tb3', 'x': '0.0', 'y': '1.0', 'z': '0.01', 'yaw': '0.0'},
        {'name': 'tb4', 'x': '1.0', 'y': '1.0', 'z': '0.01', 'yaw': '0.0'}
    ]

    # Launch Gazebo
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_file}.items()
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, 'launch', 'gzclient.launch.py')
        )
    )

    actions = [gzserver, gzclient]

    # Create a group action for each robot
    for robot in robots:
        namespace = robot['name']
        
        group = GroupAction([
            PushRosNamespace(namespace),

            # Map to Odom transform
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='map_to_odom_broadcaster',
                arguments=['0', '0', '0', '0', '0', '0', 'map', f'{namespace}/odom']
            ),

            # Robot state publisher
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'robot_description': open(os.path.join(pkg_dir, 'urdf', 'turtlebot3_burger.urdf'), 'r').read(),
                    'publish_frequency': 50.0,
                    'frame_prefix': f'{namespace}/'
                }],
                remappings=[
                    ('/tf', 'tf'),
                    ('/tf_static', 'tf_static')
                ]
            ),

            # Odom to Base Footprint transform
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='odom_to_base_footprint',
                arguments=['0', '0', '0', '0', '0', '0', f'{namespace}/odom', f'{namespace}/base_footprint']
            ),

            # Base Footprint to Base Link transform
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='base_footprint_to_base_link',
                arguments=['0', '0', '0.01', '0', '0', '0', f'{namespace}/base_footprint', f'{namespace}/base_link']
            ),

            # Base Link to LiDAR transform
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='base_link_to_laser',
                arguments=['-0.032', '0', '0.171', '0', '0', '0', f'{namespace}/base_link', f'{namespace}/base_scan']
            ),

            # Spawn robot
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                output='screen',
                arguments=[
                    '-entity', namespace,
                    '-file', os.path.join(pkg_dir, 'models', 'turtlebot3_burger', 'model.sdf'),
                    '-robot_namespace', namespace,
                    '-x', robot['x'],
                    '-y', robot['y'],
                    '-z', robot['z'],
                    '-Y', robot['yaw']
                ]
            ),

            # Scan throttler before SLAM
        # Node(
        #     package='scan_throttler',
        #     executable='throttle_scan',
        #     name='scan_throttler',
        #     parameters=[{
        #        'robot_namespace': namespace,
        #         'use_sim_time': use_sim_time,
        #          'throttle_rate': 2.0
        #     }],
        # output='screen'
        # ),

    Node(
    package='slam_toolbox',
    executable='async_slam_toolbox_node',
    name='slam_toolbox',
    output='screen',
    parameters=[{
        'use_sim_time': use_sim_time,
        'base_frame': f'{namespace}/base_footprint',
        'odom_frame': f'{namespace}/odom',
        'map_frame': 'map',
        'scan_topic': 'scan',

        # QoS settings
        'qos_overrides': {
            '/scan': {
                'history': 'keep_last',
                'depth': 10,
                'reliability': 'best_effort',
                'durability': 'volatile'
            }
        },

        # Core Configuration
        'mode': 'mapping',
        'debug_logging': True,
        'enable_interactive_mode': False,
        
        # Queue and Processing Settings
        'processor_threads': 3,              # Increased processing threads
        'max_queue_size': 30,               # Increased queue size
        
        # Timing Parameters
        'transform_timeout': 1.0,
        'tf_buffer_duration': 15.0,
        'minimum_time_interval': 0.066,     # Matches scan rate (1/15 seconds)
        'transform_publish_period': 0.066,   # Matches scan rate
        'map_update_interval': 0.5,
        
        # Processing Parameters
        'resolution': 0.05,
        'maximum_laser_range': 3.5,
        'minimum_travel_distance': 0.1,
        'minimum_travel_heading': 0.1,
        'scan_buffer_size': 30,            # Double the scan frequency
        'stack_size_to_use': 40000000,
        
        # Advanced Settings
        'optimize_by_default': False,
        'use_scan_matching': True,
        'use_scan_barycenter': True,
        
        # Thread Management
        'threads_processing': 3,
        'threads_measurements': 3
    }],
    remappings=[
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static'),
        ('map', 'map'),
        ('scan', 'scan')
    ]
)


    
        ])
        
        actions.append(group)

    # Map merge nodes with proper sequencing
    merge_nodes = GroupAction([
        # Merge maps for first pair of robots (tb1 + tb2)
        Node(
            package='merge_map',
            executable='merge_map',
            name='merge_1_2',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[
                ("/map1", "/tb1/map"),
                ("/map2", "/tb2/map"),
                ("/merge_map", "/merge_1_2")
            ],
            #Qos
            ros_arguments=['--qos-overrides', '/tb1/map:=reliable_transient']
        ),

        # Merge maps for second pair of robots (tb3 + tb4)
        Node(
            package='merge_map',
            executable='merge_map',
            name='merge_3_4',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[
                ("/map1", "/tb3/map"),
                ("/map2", "/tb4/map"),
                ("/merge_map", "/merge_3_4")
            ]
        ),

        # Final merge of the two paired merges
        Node(
            package='merge_map',
            executable='merge_map',
            name='merge_final',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[
                ("/map1", "/merge_1_2"),
                ("/map2", "/merge_3_4"),
                ("/merge_map", "/merged_map")
            ]
        )
    ])

    actions.append(merge_nodes)

    return LaunchDescription(actions)