import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    # Package directories
    pkg_dir = get_package_share_directory('slam')
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # Paths
    xacro_file = os.path.join(pkg_dir, 'urdf', 'car.urdf.xacro')
    controller_config = os.path.join(pkg_dir, 'config', 'controller.yaml')
    slam_config = os.path.join(pkg_dir, 'config', 'mapper_params_online_async.yaml')
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    
    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
        
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(nav2_bringup_dir, 'params', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value='',
        description='Full path to map yaml file to load')
    
    # Get URDF via xacro
    robot_description = {'robot_description': Command(['xacro ', xacro_file])}

    # RViz Config file
    rviz_config_file = os.path.join(pkg_dir, 'config', 'slam.rviz')

    # Define nodes
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}],
    )

    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        arguments=['-topic', 'robot_description', 
                  '-entity', 'diff_drive_robot',
                  '-x', '0.0',
                  '-y', '0.0',
                  '-z', '0.1']
    )

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

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )

    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
    )

    # Delay diff_drive_spawner until joint_state_broadcaster_spawner has finished
    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[diff_drive_spawner],
        )
    )

    # Create LaunchDescription and add actions
    ld = LaunchDescription([
        declare_use_sim_time_cmd,
        declare_params_file_cmd,
        declare_map_yaml_cmd,

        # Launch Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros_dir, 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'world': ''
            }.items()
        ),

        # Core robot nodes
        robot_state_publisher_node,
        spawn_entity_node,
        controller_manager_node,
        joint_state_broadcaster_spawner,
        delayed_diff_drive_spawner,

        # SLAM Toolbox
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                slam_config,
                {'use_sim_time': use_sim_time}
            ],
        ),

        # Operator node
        Node(
            package='slam',
            executable='operatorNode',
            name='operator_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),

        # Launch Nav2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': params_file
            }.items()
        ),
    ])
    
    return ld