import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import torch
import torch.nn as nn
import torch.nn.functional as F
import numpy as np
import os
from collections import deque
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry  # Import Odometry message type
import sys
# PKG DIR
pkg_dir = get_package_share_directory('slam')

# Reward path
sys.path.append(os.path.join(pkg_dir, 'reward.py'))

from reward_function import RewardLiDAR



 #Actor Network
class Actor(nn.Module):
    def __init__(self, lidar_dim, state_dim, action_dim, max_action):
        super(Actor, self).__init__()
        self.lidar_encoder = nn.Sequential(
            nn.Linear(lidar_dim, 128),  # Encode LiDAR readings
            nn.ReLU(),
            nn.Linear(128, 128),
            nn.ReLU()
        )
        self.state_encoder = nn.Sequential(
            nn.Linear(state_dim, 128),  # Encode additional state inputs (e.g., velocity, heading)
            nn.ReLU(),
            nn.Linear(128, 128),
            nn.ReLU()
        )
        self.fusion = nn.Sequential(
            nn.Linear(128 + 128, 256),  # Combine encoded LiDAR and state features
            nn.ReLU(),
            nn.Linear(256, 256),
            nn.ReLU(),
            nn.Linear(256, action_dim),
            nn.Tanh()
        )
        self.max_action = max_action

    def forward(self, lidar, state):
        lidar_features = self.lidar_encoder(lidar)
        state_features = self.state_encoder(state)
        combined = torch.cat([lidar_features, state_features], dim=-1)
        return self.max_action * self.fusion(combined)


# Critic Network
class Critic(nn.Module):
    def __init__(self, lidar_dim, state_dim, action_dim):
        super(Critic, self).__init__()
        self.lidar_encoder = nn.Sequential(
            nn.Linear(lidar_dim, 128),  # Encode LiDAR readings
            nn.ReLU(),
            nn.Linear(128, 128),
            nn.ReLU()
        )
        self.state_encoder = nn.Sequential(
            nn.Linear(state_dim, 128),  # Encode additional state inputs (e.g., velocity, heading)
            nn.ReLU(),
            nn.Linear(128, 128),
            nn.ReLU()
        )
        self.q1 = nn.Sequential(
            nn.Linear(128 + 128 + action_dim, 256),  # Combine encoded features and action
            nn.ReLU(),
            nn.Linear(256, 256),
            nn.ReLU(),
            nn.Linear(256, 1)
        )
        self.q2 = nn.Sequential(
            nn.Linear(128 + 128 + action_dim, 256),
            nn.ReLU(),
            nn.Linear(256, 256),
            nn.ReLU(),
            nn.Linear(256, 1)
        )

    def forward(self, lidar, state, action):
        lidar_features = self.lidar_encoder(lidar)
        state_features = self.state_encoder(state)
        combined = torch.cat([lidar_features, state_features, action], dim=-1)
        q1 = self.q1(combined)
        q2 = self.q2(combined)
        return q1, q2

    def q1_only(self, lidar, state, action):
        lidar_features = self.lidar_encoder(lidar)
        state_features = self.state_encoder(state)
        combined = torch.cat([lidar_features, state_features, action], dim=-1)
        return self.q1(combined)
    def forward(self, state, action):
        sa = torch.cat([state, action], dim=-1)
        q1 = self.q1(sa)
        q2 = self.q2(sa)
        return q1, q2

    def q1_only(self, state, action):
        sa = torch.cat([state, action], dim=-1)
        return self.q1(sa)


class Controller(Node):

    def __init__(self):
        super().__init__("controller")
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        
        # State and action dimensions
        self.state_dim = 6  # Update this to match your state dimension
        self.action_dim = 2   # Update this to match your action dimension
        self.max_action = 1.0 # Update this to match your max action value

        # Pkg dir
        pkg_dir = get_package_share_directory('slam')

        # File paths for pre-trained models
        actor_model_path = os.path.join(pkg_dir, 'actor_model.pt')
        critic_model_path = os.path.join(pkg_dir, 'critic_model.pt')

        # Actor network
        self.actor = Actor(self.state_dim, self.action_dim, self.max_action).to(self.device)
        if os.path.exists(actor_model_path):
            self.actor.load_state_dict(torch.load(actor_model_path))  # Load pre-trained Actor model
        else:
            self.get_logger().warn(f"Actor model file {actor_model_path} not found. Ensure the model exists.")
        self.actor.eval()

        # Critic network
        self.critic = Critic(self.state_dim, self.action_dim).to(self.device)
        if os.path.exists(critic_model_path):
            self.critic.load_state_dict(torch.load(critic_model_path))  # Load pre-trained Critic model
        else:
            self.get_logger().warn(f"Critic model file {critic_model_path} not found. Ensure the model exists.")
        self.critic.eval()  # Optional: set to evaluation mode initially

        # Optimizer for online updates
        self.actor_optimizer = torch.optim.Adam(self.actor.parameters(), lr=3e-4)
        self.critic_optimizer = torch.optim.Adam(self.critic.parameters(), lr=3e-4)

        # Training parameters
        self.gamma = 0.99
        self.tau = 0.005
        self.replay_buffer = deque(maxlen=10000)
        self.batch_size = 64

        # QoS profiles and subscribers/publishers
        laser_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        
        twist_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Subscribers
        self.twist_sub_ = self.create_subscription(
            Twist, 
            "/cmd_vel", 
            self.incoming, 
            twist_qos
        )

        # Target Pose Subscriber
        self.target_sub_ = self.create_subscription(
            Pose,
            'target_pose',
            self.target_callback,
            10
        )

         #Subscribe to odometry topic to get the robot's position
        self.odom_sub_ = self.create_subscription(
             Odometry,
             "/odom",  # Replace with your odometry topic if different
             self.odom_callback,
             10
        ) 

        self.twist_pub_ = self.create_publisher(
            Twist,
            "/diff_drive_controller/cmd_vel_unstamped",
            10
        )

        self.get_logger().info("Controller Node Started Successfully")

        # Publishers
        self.twist_pub_ = self.create_publisher(
            Twist, 
            "/diff_drive_controller/cmd_vel_unstamped", 
            twist_qos
        )
        

        self.get_logger().info("Controller Node Started Successfully")

        def odom_callback(self, msg: Odometry):
        #"""Callback to handle odometry data and update the current position"""
        # Extract the position from the Odometry message
         self.current_position = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
         self.get_logger().info(f"Current Position: x={self.current_position[0]}, y={self.current_position[1]}")

    def target_callback(self, msg: Pose):
        """Callback to handle target pose and calculate action"""
        self.get_logger().info("Target Received successfully")
        target_position = np.array([msg.position.x, msg.position.y])

        # Use current_position (updated by odometry) and target_position
        delta_position = target_position - self.current_position
        
        # You would need to form a state based on this (e.g., distance to target, heading)
        state = np.concatenate([delta_position])

        # Get the action from the Actor network
        action = self.get_action(state)

        # Create Twist message (e.g., for linear and angular velocity)
        twist_msg = Twist()
        twist_msg.linear.x = action[0]
        twist_msg.angular.z = action[1]

        # Publish the command
        self.twist_pub_.publish(twist_msg)

    def get_action(self, state):
    #"""Get action from actor network"""
     with torch.no_grad():
        # Pass normalized state vector
        state_tensor = torch.FloatTensor(state).to(self.device)
        lidar_placeholder = torch.zeros((1, self.state_dim)).to(self.device)  # Use zeros if no LiDAR data
        action = self.actor(lidar_placeholder, state_tensor).cpu().numpy()
     return action


    def evaluate_action(self, state, action):
        """Evaluate the action using the Critic"""
        with torch.no_grad():
            state = torch.FloatTensor(state).to(self.device)
            action = torch.FloatTensor(action).to(self.device)
            q1, q2 = self.critic(state, action)
        return torch.min(q1, q2).item()  # Return the smaller Q-value

    def add_to_replay_buffer(self, state, action, reward, next_state, done):
        self.replay_buffer.append((state, action, reward, next_state, done))
        if len(self.replay_buffer) > 10000:  # Limit buffer size
            self.replay_buffer.pop(0)

    def sample_replay_buffer(self):
        """Sample a batch from the replay buffer"""
        batch = np.random.choice(self.replay_buffer, self.batch_size)
        state, action, reward, next_state, done = zip(*batch)
        return torch.FloatTensor(state), torch.FloatTensor(action), torch.FloatTensor(reward), torch.FloatTensor(next_state), torch.FloatTensor(done)

    def train(self):
        if len(self.replay_buffer) < self.batch_size:
            return  # Not enough data to train

        state, action, reward, next_state, done = self.sample_replay_buffer()

        # Update Critic
        with torch.no_grad():
            next_action = self.actor(next_state)
            target_q1, target_q2 = self.critic(next_state, next_action)
            target_q = reward + (1 - done) * self.gamma * torch.min(target_q1, target_q2)

        current_q1, current_q2 = self.critic(state, action)
        critic_loss = F.mse_loss(current_q1, target_q) + F.mse_loss(current_q2, target_q)

        self.critic_optimizer.zero_grad()
        critic_loss.backward()
        self.critic_optimizer.step()

        # Optionally update the Actor network using the pre-trained Critic
        actor_loss = -self.critic.q1_only(state, self.actor(state)).mean()
        self.actor_optimizer.zero_grad()
        actor_loss.backward()
        self.actor_optimizer.step()

    def incoming(self, msg: Twist):
        # Forward the twist message to the controller
        self.twist_pub_.publish(msg)

    def target_callback(self, msg: Pose):
        # Compute the necessary action based on the target pose
        self.get_logger().info("Target Received successfully")
        target_position = np.array([msg.position.x, msg.position.y])
        target_orientation = np.array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        
        # Use your state (e.g., current robot pose) and target to calculate the desired action
        # For example, you can calculate the relative position (delta position)
        current_position = np.array([0, 0])  # Get this from odometry or sensors
        delta_position = target_position - current_position
        
        # You would need to form a state based on this (e.g., distance to target, heading)
        state = np.concatenate([delta_position, target_orientation])

        # Get the action from the Actor network
        action = self.get_action(state)
        
        # Scale the action values to match robot's velocity range
    max_linear_speed = 0.5  # Maximum linear speed (adjust based on your robot)
    max_angular_speed = 1.0  # Maximum angular speed (adjust based on your robot)

    twist_msg = Twist()
    # Scale linear speed (action[0]) and angular speed (action[1]) based on max limits
    twist_msg.linear.x = action[0] * max_linear_speed
    twist_msg.angular.z = action[1] * max_angular_speed
        
        # Publish the command
        self.twist_pub_.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
