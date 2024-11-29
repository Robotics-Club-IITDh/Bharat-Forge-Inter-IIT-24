import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    # Get the launch directory
    pkg_dir = get_package_share_directory('slam')
    
    # Declare launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='robot_1',
        description='Namespace for the robot'
    )
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    # Get configurations
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Create proper frame substitutions
    base_frame = PythonExpression(['"', namespace, ' + "/base_link"', '"'])
    odom_frame = PythonExpression(['"', namespace, ' + "/odom"', '"'])
    
    # SLAM node
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        namespace=namespace,
        output='screen',
        parameters=[
            os.path.join(pkg_dir, 'config', 'mapper_params_online_async.yaml'),
            {
                'use_sim_time': use_sim_time,
                'base_frame': base_frame,
                'odom_frame': odom_frame,
                'scan_topic': 'scan'
            }
        ]
    )
    
    # Return launch description
    return LaunchDescription([
        declare_namespace_cmd,
        declare_use_sim_time_cmd,
        slam_toolbox_node
    ])

