#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan  # Import LaserScan message
from geometry_msgs.msg import Twist
import torch
import numpy as np
import os
import sys
from ament_index_python.packages import get_package_share_directory

# Get the package share directory for 'slam'
package_share_dir = get_package_share_directory('slam')

# Point to the models directory within the package
models_dir = os.path.join(package_share_dir, 'models')

# Add the models directory to the Python module search path
sys.path.append(models_dir)

from ppo_model import PPOModel  # Import the PPO model
from reward import RewardLiDAR  # Import the RewardLiDAR class

class PPOController(Node):
    def __init__(self):
        super().__init__("ppo_controller")

        # PPO weight site
        pkg_dir = get_package_share_directory("slam")
        weights = os.path.join(pkg_dir, "models", "ppo_weights.pth")
        
        # Load PPO model
        self.ppo_model = PPOModel(state_dim=363, action_dim=2)  # Update dimensions as needed
        self.ppo_model.load_state_dict(torch.load(weights))
        self.ppo_model.eval()

        self.get_logger().info("Loaded PPO model and weights")

        # Initialize RewardLiDAR
        self.reward_lidar = RewardLiDAR(
            num_beams=360,
            max_range=10.0, 
            collision_threshold=0.5,
            goal_position=(5.0, 5.0),
            collision_penalty=100.0,
            action_cost=0.1,
            raycast_cost=20.0,
            goal_reward_scale=50.0,
            fov=np.pi
        )
        self.get_logger().info("Initialized RewardLiDAR")

        # QoS for reliable communication
        self.declare_parameter("namespace", "")
        self.namespace = self.get_parameter("namespace").get_parameter_value().string_value
        self.get_logger().info(f"Namespace: {self.namespace}")

        # Subscribers
        self.map_subscription = self.create_subscription(
            OccupancyGrid, "/merge_map", self.map_callback, 10
        )
        self.odom_subscription = self.create_subscription(
            Odometry, f"/{self.namespace}/odom", self.odom_callback, 10
        )
        self.scan_subscription = self.create_subscription(
            LaserScan, f"/{self.namespace}/scan", self.scan_callback, 10
        )

        # Publisher
        self.velocity_publisher = self.create_publisher(
            Twist, f"/{self.namespace}/cmd_vel", 10
        )

        # State variables
        self.map_data = None
        self.current_position = None
        self.current_orientation = None
        self.current_lidar_data = None  # Store LiDAR data here
        
        # Timer for action inference
        self.create_timer(0.1, self.control_loop)  # 10 Hz
        self.get_logger().info("PPO Controller with Reward Function Initialized")

    def map_callback(self, msg):
        """Handle map updates."""
        self.map_data = list(msg.data)

    def odom_callback(self, msg):
        """Handle odometry updates."""
        self.current_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
        )
        orientation = msg.pose.pose.orientation
        self.current_orientation = self.quaternion_to_yaw(
            orientation.x, orientation.y, orientation.z, orientation.w
        )

    def scan_callback(self, msg):

        # Convert ranges to a NumPy array
        self.current_lidar_data = {
            "ranges": np.array(msg.ranges, dtype=np.float32),  # Use NumPy for efficient processing
            "intensities": np.array(msg.intensities, dtype=np.float32) if msg.intensities else np.array([]),
            "angle_min": msg.angle_min,
            "angle_max": msg.angle_max,
            "angle_increment": msg.angle_increment,
            "range_min": msg.range_min,
            "range_max": msg.range_max
        }
        # Debug: Print the first 10 range values
        self.get_logger().debug(f"LiDAR Ranges (first 10): {self.current_lidar_data['ranges'][:10]}")
    

    @staticmethod
    def quaternion_to_yaw(x, y, z, w):
        """Convert quaternion to yaw angle."""
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return np.arctan2(siny_cosp, cosy_cosp)

    def control_loop(self):
    if self.map_data is None or self.current_position is None or self.current_lidar_data is None:
        self.get_logger().info("Waiting for all data inputs to be available...")
        return

    # Prepare state and LiDAR data
    state = self.prepare_state()
    state_tensor = torch.tensor(state, dtype=torch.float32).unsqueeze(0)
    
    # Process LiDAR data
    lidar_ranges = self.current_lidar_data['ranges']
    if not isinstance(lidar_ranges, np.ndarray):
        lidar_ranges = np.array(lidar_ranges, dtype=np.float32)
    
    # Handle inf and nan values in LiDAR data
    min_range = self.current_lidar_data['range_min']
    max_range = self.reward_lidar.max_range
    lidar_ranges = np.clip(
        np.nan_to_num(lidar_ranges, nan=max_range, posinf=max_range, neginf=min_range),
        min_range,
        max_range
    )

    # Check for obstacles
    collision_threshold = self.reward_lidar.collision_threshold
    min_distance = np.min(lidar_ranges)
    is_obstacle_close = min_distance < collision_threshold * 2  # Double the collision threshold for safety

    # Get base action from PPO model
    with torch.no_grad():
        base_linear_vel, base_angular_vel = self.ppo_model.get_action(state_tensor)

    # Adjust velocities based on obstacles
    if is_obstacle_close:
        # Get the index of minimum distance reading
        min_distance_idx = np.argmin(lidar_ranges)
        num_beams = len(lidar_ranges)
        
        # Determine if obstacle is more to the left or right
        is_obstacle_left = min_distance_idx < num_beams / 2
        
        # Modify velocities for obstacle avoidance
        linear_vel = 0.0  # Stop forward motion
        
        # Turn away from obstacle
        if is_obstacle_left:
            angular_vel = -0.3  # Turn right
        else:
            angular_vel = 0.3   # Turn left
            
        self.get_logger().info(f"Obstacle detected! Distance: {min_distance:.2f}, Adjusting velocities")
    else:
        # Use base velocities when no obstacles
        linear_vel = base_linear_vel
        angular_vel = base_angular_vel

    # Compute reward
    action = np.array([linear_vel, angular_vel])
    try:
        reward = self.reward_lidar.computeRewardFromLiDAR(
            robot_state=[*self.current_position, self.current_orientation],
            lidar_ranges=lidar_ranges,
            u=action
        )
        self.get_logger().info(f"Reward: {reward:.2f}")
    except Exception as e:
        self.get_logger().error(f"Error computing reward: {str(e)}")
        return

    # Publish velocity commands
    cmd_msg = Twist()
    cmd_msg.linear.x = float(linear_vel)
    cmd_msg.angular.z = float(angular_vel)
    self.velocity_publisher.publish(cmd_msg)
    
    # Log final velocities
    self.get_logger().debug(
        f"Published velocities: linear={linear_vel:.3f}, angular={angular_vel:.3f}, "
        f"Obstacle detected: {is_obstacle_close}"
    )


    def prepare_state(self):
        """Prepare state vector for PPO."""
        # Use map data, position, and orientation as inputs
        state = self.map_data[:360]  # Example: Use first 360 cells of map
        state.extend([self.current_position[0], self.current_position[1], self.current_orientation])
        return state

def main(args=None):
    rclpy.init(args=args)
    node = PPOController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()