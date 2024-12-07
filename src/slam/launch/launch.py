import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch.event_handlers import OnProcessExit
from nav2_common.launch import RewrittenYaml
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Declare launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='robot_1',
        description='Namespace for the robot'
    )
    
    declare_x_pos_cmd = DeclareLaunchArgument(
        'x_pos',
        default_value='0.0',
        description='X position for robot spawning'
    )
    
    declare_y_pos_cmd = DeclareLaunchArgument(
        'y_pos',
        default_value='0.0',
        description='Y position for robot spawning'
    )
    
    declare_z_pos_cmd = DeclareLaunchArgument(
        'z_pos',
        default_value='0.1',
        description='Z position for robot spawning'
    )
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    # Get configurations
    namespace = LaunchConfiguration('namespace')
    x_pos = LaunchConfiguration('x_pos')
    y_pos = LaunchConfiguration('y_pos')
    z_pos = LaunchConfiguration('z_pos')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Package directories
    pkg_dir = get_package_share_directory('slam')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    
    # Paths
    xacro_file = os.path.join(pkg_dir, 'urdf', 'robot.urdf.xacro')
    controller_config = os.path.join(pkg_dir, 'config', 'controller_robot_1.yaml')
    slam_config = os.path.join(pkg_dir, 'config', 'mapper_params_online_async.yaml')
    nav2_params_file = os.path.join(nav2_bringup_dir, 'params', 'nav2_params.yaml')
    



    # Get URDF via xacro
    robot_description_config = Command([
        'xacro ', xacro_file,
        ' namespace:=', namespace,
        ' x:=', x_pos,
        ' y:=', y_pos,
        ' z:=', z_pos
    ])
    
    robot_description = {'robot_description': robot_description_config}

    # Define nodes
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=namespace,
        output='screen',
        parameters=[
            robot_description,
            {
                'use_sim_time': use_sim_time,
                # 'frame_prefix': [namespace, '/'],
                # Add explicit QoS settings for static transforms
                'qos_overrides./tf_static.publisher.durability': 'transient_local',
                'qos_overrides./tf_static.publisher.reliability': 'reliable'
            }
        ],
        arguments=[]
    )   

    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        namespace=namespace,
        output='screen',
        arguments=[
            '-topic', ['/', namespace, '/robot_description'],
            '-entity', namespace,
            '-x', x_pos,
            '-y', y_pos,
            '-z', z_pos
        ]
    )

    # ROS2 Control related nodes
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            controller_config,
            robot_description,
            {'use_sim_time': use_sim_time}
        ],
        output='screen',
    )

    # Joint State Publisher
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        namespace=namespace,
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Controller Spawners
    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        namespace=namespace,
        arguments=['diff_drive_controller'],
        output='screen'
    )

    joint_broad_spawner = Node(
        package='controller_manager',
        executable='spawner',
        namespace=namespace,
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    # Create event handlers for startup sequence
    spawn_to_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity_node,
            on_exit=[controller_manager_node],
        )
    )

    controller_to_spawners = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=controller_manager_node,
            on_exit=[joint_broad_spawner, diff_drive_spawner],
        )
    )

    camera_processor = Node(
        package='slam',
        executable='camera_node',
        name='camera_node',
        namespace=namespace,
        parameters=[{
            'namespace': namespace
        }],
        output='screen'
    )

    operator_node = Node(
        package='slam',
        executable='operator_node',
        name='operator_node',
        namespace=namespace,
        parameters=[{
            'namespace': namespace
        }],
        output='screen'
    )
    
    # Configure the SLAM node with proper namespacing
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',  # This will be prefixed with the namespace
        namespace=namespace,  # Explicitly set the namespace
        output='screen',
        parameters=[
            slam_config,
            {
                'use_sim_time': use_sim_time,
                'scan_topic': PythonExpression(['"/', namespace, '/scan"']),
                'base_frame': PythonExpression(['"', namespace, '/base_link"']),
                'odom_frame': PythonExpression(['"', namespace, '/odom"']),
            }
        ],
        remappings=[
            ('/map', ['/', namespace, '/map'])
        ]
    )

    astar_controller_node = Node(
        package='slam',
        executable='astar_controller',
        name='astar_controller',
        namespace=namespace,
        parameters=[{
            'namespace': namespace,
            'use_sim_time': use_sim_time
        }],
        output='screen'
        )


    nav2_bringup_launch = IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
                    ]), 
                    launch_arguments={
                        'namespace': namespace,
                        'use_namespace': 'True',
                        'params_file': nav2_params_file,
                        'use_sim_time': 'True',
                        'autostart': 'True',
                        'map_topic': '/merged_map',
                    }.items()
                )

    # Create and return launch description
    ld = LaunchDescription([
        # Launch Arguments
        declare_namespace_cmd,
        declare_x_pos_cmd,
        declare_y_pos_cmd,
        declare_z_pos_cmd,
        declare_use_sim_time_cmd,
        
        # Core nodes
        robot_state_publisher_node,
        joint_state_publisher_node,
        
        # Spawn entity
        spawn_entity_node,
        
        # Controller sequence
        camera_processor,
        spawn_to_controller,
        controller_to_spawners,
        slam_toolbox_node,
        operator_node,
        # astar_controller_node,
        # nav2_bringup_launch,

    ])
    
    return ld
