#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
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
        self.ppo_model = PPOModel(state_dim=4, action_dim=2)  # Update dimensions as needed
        self.ppo_model.load_state_dict(torch.load(weights))  # Load trained weights
        self.ppo_model.eval()  # Set model to evaluation mode

        # Initialize RewardLiDAR
        self.reward_lidar = RewardLiDAR(
            num_beams=360,  # Match the LiDAR's resolution
            max_range=10.0,
            collision_threshold=0.5,
            goal_position=(5.0, 5.0),  # Example goal position
            collision_penalty=100.0,
            action_cost=0.1,
            raycast_cost=20.0,
            goal_reward_scale=50.0,
            fov=np.pi
        )

        # QoS for reliable communication
        self.declare_parameter("namespace", "")
        self.namespace = self.get_parameter("namespace").get_parameter_value().string_value
        
        # Subscribers
        self.map_subscription = self.create_subscription(
            OccupancyGrid, "/merge_map", self.map_callback, 10
        )
        self.odom_subscription = self.create_subscription(
            Odometry, f"/{self.namespace}/odom", self.odom_callback, 10
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

    @staticmethod
    def quaternion_to_yaw(x, y, z, w):
        """Convert quaternion to yaw angle."""
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return np.arctan2(siny_cosp, cosy_cosp)

    def control_loop(self):
        """Compute actions, rewards, and publish velocity commands."""
        if self.map_data is None or self.current_position is None or self.current_lidar_data is None:
            return  # Wait for data
        
        # Prepare state vector for PPO
        state = self.prepare_state()
        state_tensor = torch.tensor(state, dtype=torch.float32).unsqueeze(0)
        self.get_logger().info(f"State tensor: {state_tensor}")
        # Get action from PPO model
        with torch.no_grad():
            action = self.ppo_model.get_action(state_tensor)
        self.get_logger().info(f"Computed action: {action}")
        # Compute reward
        reward = self.reward_lidar.computeRewardFromLiDAR(
            robot_state=[*self.current_position, self.current_orientation],
            lidar_ranges=self.current_lidar_data,
            u=action
        )
        self.get_logger().info(f"Reward: {reward:.2f}")
        self.get_logger().info(f"Publishing velocities: linear={action[0]}, angular={action[1]}")
        # Publish velocity commands
        cmd_msg = Twist()
        cmd_msg.linear.x = action[0]  # Linear velocity
        cmd_msg.angular.z = action[1]  # Angular velocity
        self.velocity_publisher.publish(cmd_msg)

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
