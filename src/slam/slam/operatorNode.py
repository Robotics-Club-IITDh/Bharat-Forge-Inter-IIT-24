#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf_transformations import euler_from_quaternion
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import torch
import numpy as np
import os
from .ppo_model import PPOModel  # Import the existing PPO model
from .exploration_reward import ExplorationReward  # Import the exploration reward system

class OperatorNode(Node):
    def __init__(self):
        super().__init__('operator_node')
        
        # Initialize device
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        
        # Initialize model
        self.state_dim = 8  # [x, y, cos(θ), sin(θ), goal_dist, goal_angle, linear_vel, angular_vel]
        self.lidar_dim = 8  # 8 sectors of processed LiDAR data
        self.ppo_model = PPOModel(self.state_dim, self.lidar_dim).to(self.device)
        
        # Load model weights
        weights_path = "ppo_weights.pth"
        if os.path.exists(weights_path):
            self.ppo_model.load_state_dict(torch.load(weights_path, map_location=self.device))
            self.get_logger().info(f"Model weights loaded from {weights_path}")
        else:
            self.get_logger().warn(f"No weights found at {weights_path}")
        
        self.ppo_model.eval()
        
        # Initialize exploration parameters
        self.exploration_phase = True
        self.exploration_time = 60.0  # Time in seconds for initial exploration
        self.start_time = self.get_clock().now()
        self.exploration_steps = 0
        self.random_action_prob = 0.3  # Probability of taking random action during exploration
        
        # Initialize reward system
        self.reward_system = ExplorationReward(
            num_beams=self.lidar_dim,
            max_range=3.5,
            collision_threshold=0.3
        )
        
        # Movement control parameters
        self.min_straight_distance = 1.0  # Minimum distance for straight movement
        self.direction_change_threshold = 0.5  # Threshold for changing direction
        self.current_action = None
        self.action_repeat_count = 0
        self.max_action_repeat = 20  # Maximum steps to repeat an action
        self.collision_recovery_steps = 10  # Steps to take during collision recovery
        self.recovery_count = 0
        
        # Initialize state variables
        self.robot_position = np.zeros(2)
        self.robot_orientation = 0.0
        self.robot_linear_vel = 0.0
        self.robot_angular_vel = 0.0
        self.goal_position = np.zeros(2)
        self.latest_scan = None
        
        # Position history for recovery
        self.position_history = []
        self.max_history = 50
        
        # Set up QoS profiles
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Create publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(
            Twist, 
            'cmd_vel',
            reliable_qos
        )
        
        self.metrics_pub = self.create_publisher(
            String,
            'exploration_metrics',
            10
        )
        
        self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            sensor_qos
        )
        
        self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            sensor_qos
        )
        
        self.create_subscription(
            PoseStamped,
            'goal_pose',
            self.goal_callback,
            reliable_qos
        )
        
        # Create control loop timer (10 Hz)
        self.create_timer(0.1, self.control_loop)
        
        # Status variables
        self.goal_reached_threshold = 0.3
        self.is_goal_active = False
        self.in_recovery_mode = False
        
        # Debug flags
        self.debug_mode = True
        
        self.get_logger().info("Enhanced operator node initialized")

    def odom_callback(self, msg):
        """Update robot state from odometry"""
        # Position
        self.robot_position[0] = msg.pose.pose.position.x
        self.robot_position[1] = msg.pose.pose.position.y
        
        # Orientation (convert quaternion to euler)
        orientation_q = msg.pose.pose.orientation
        _, _, self.robot_orientation = euler_from_quaternion(
            [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        )
        
        # Velocities
        self.robot_linear_vel = msg.twist.twist.linear.x
        self.robot_angular_vel = msg.twist.twist.angular.z
        
        if self.debug_mode:
            self.get_logger().debug(
                f"Odom update - Pos: [{self.robot_position[0]:.2f}, {self.robot_position[1]:.2f}], "
                f"Orientation: {self.robot_orientation:.2f}, "
                f"Vel: [{self.robot_linear_vel:.2f}, {self.robot_angular_vel:.2f}]"
            )

    def update_position_history(self):
        """Update position history for recovery"""
        current_pos = np.array([self.robot_position[0], self.robot_position[1]])
        self.position_history.append(current_pos)
        if len(self.position_history) > self.max_history:
            self.position_history.pop(0)

    def get_recovery_position(self):
        """Get position to return to after collision"""
        if len(self.position_history) < 10:
            return None
        return self.position_history[-10]  # Return to position 10 steps ago

    def process_lidar_data(self, ranges):
        """Process LiDAR data into sectors"""
        if ranges is None:
            return np.zeros(self.lidar_dim)
            
        # Convert inf values to max_range
        processed_ranges = np.array(ranges)
        processed_ranges[np.isinf(processed_ranges)] = 3.5  # max range
        
        # Split into sectors
        sectors = np.array_split(processed_ranges, self.lidar_dim)
        sector_mins = np.array([np.min(sector) for sector in sectors])
        
        # Normalize to [0, 1]
        normalized_sectors = np.clip(sector_mins / 3.5, 0, 1)
        
        return normalized_sectors

    def scan_callback(self, msg):
        """Store and process latest LiDAR scan"""
        self.latest_scan = msg.ranges
        if self.debug_mode:
            self.get_logger().debug(f"Received scan with {len(msg.ranges)} points")

    def goal_callback(self, msg):
        """Update goal position"""
        self.goal_position[0] = msg.pose.position.x
        self.goal_position[1] = msg.pose.position.y
        self.is_goal_active = True
        self.get_logger().info(f"New goal received: [{self.goal_position[0]:.2f}, {self.goal_position[1]:.2f}]")

    def calculate_safe_direction(self, scan):
        """Calculate safe direction based on LiDAR data"""
        if scan is None:
            return 0.0
        
        # Convert scan to numpy array and handle inf values
        scan_array = np.array(scan)
        scan_array[np.isinf(scan_array)] = self.reward_system.max_range
        
        # Find the direction with most open space
        sector_size = len(scan_array) // 8
        sectors = np.array_split(scan_array, 8)
        sector_averages = [np.mean(sector) for sector in sectors]
        best_sector = np.argmax(sector_averages)
        
        # Convert sector index to angle
        angle = (best_sector - 3.5) * (np.pi / 4)  # -pi/2 to pi/2
        return angle

    def check_exploration_phase(self):
        """Check if we should still be in exploration phase"""
        current_time = self.get_clock().now()
        elapsed_time = (current_time - self.start_time).nanoseconds / 1e9
        
        if self.exploration_phase and elapsed_time > self.exploration_time:
            self.exploration_phase = False
            self.get_logger().info("Exploration phase complete, switching to normal operation")
            
        return self.exploration_phase

    def publish_metrics(self):
        """Publish exploration metrics"""
        stats = self.reward_system.get_exploration_stats()
        metrics_msg = String()
        metrics_msg.data = f"Explored cells: {stats['explored_cells']}, Coverage: {stats['exploration_coverage']:.2f}m²"
        self.metrics_pub.publish(metrics_msg)

    def get_exploration_action(self):
        """Generate exploration action with improved consistency"""
        # If in recovery mode, generate recovery action
        if self.in_recovery_mode:
            if self.recovery_count >= self.collision_recovery_steps:
                self.in_recovery_mode = False
                self.recovery_count = 0
            else:
                self.recovery_count += 1
                return self.reward_system.get_recovery_action()
        
        # Check if we should continue current action
        if (self.current_action is not None and 
            self.action_repeat_count < self.max_action_repeat):
            
            # Check if path is still clear
            is_obstacle_close, _ = self.reward_system.check_obstacle_proximity(self.latest_scan)
            
            if not is_obstacle_close:
                self.action_repeat_count += 1
                return self.current_action
        
        # Generate new action
        if np.random.random() < self.random_action_prob:
            # Calculate safe direction based on LiDAR
            safe_angle = self.calculate_safe_direction(self.latest_scan)
            
            # Generate action with preference for straight movement
            if np.random.random() < 0.7:  # 70% chance for forward movement
                linear_vel = np.random.uniform(0.2, 0.5)
                angular_vel = safe_angle * 0.5  # Reduced angular velocity for smoother turns
            else:
                linear_vel = np.random.uniform(-0.2, 0.3)
                angular_vel = np.random.uniform(-0.8, 0.8)
        else:
            # Use model's action but add minimal noise for exploration
            state = self.prepare_state_input()
            with torch.no_grad():
                action, _ = self.ppo_model.get_action(state, deterministic=True)
            
            linear_vel = float(action[0, 0].cpu()) + np.random.normal(0, 0.05)
            angular_vel = float(action[0, 1].cpu()) + np.random.normal(0, 0.1)
        
        # Clip velocities
        linear_vel = np.clip(linear_vel, -0.3, 0.5)
        angular_vel = np.clip(angular_vel, -1.0, 1.0)
        
        # Update current action
        self.current_action = np.array([linear_vel, angular_vel])
        self.action_repeat_count = 0
        
        return self.current_action

    def prepare_state_input(self):
        """Prepare the state input for the PPO model"""
        # Calculate goal distance and angle
        goal_vector = self.goal_position - self.robot_position
        goal_distance = np.linalg.norm(goal_vector)
        goal_angle = np.arctan2(goal_vector[1], goal_vector[0]) - self.robot_orientation
        
        # Normalize goal angle to [-π, π]
        goal_angle = np.arctan2(np.sin(goal_angle), np.cos(goal_angle))
        
        # Process LiDAR data
        lidar_features = self.process_lidar_data(self.latest_scan)
        
        # Combine all state features
        state = np.array([
            self.robot_position[0],
            self.robot_position[1],
            np.cos(self.robot_orientation),
            np.sin(self.robot_orientation),
            goal_distance,
            goal_angle,
            self.robot_linear_vel,
            self.robot_angular_vel,
            *lidar_features  # Add processed LiDAR features
        ])
        
        return torch.FloatTensor(state).unsqueeze(0).to(self.device)

    def check_goal_reached(self):
        """Check if current goal has been reached"""
        if not self.is_goal_active:
            return False
            
        distance_to_goal = np.linalg.norm(
            self.robot_position - self.goal_position
        )
        
        if distance_to_goal < self.goal_reached_threshold:
            self.is_goal_active = False
            self.get_logger().info("Goal reached!")
            return True
            
        return False

    def control_loop(self):
        """Main control loop with exploration and collision recovery"""
        try:
            # Update position history
            self.update_position_history()
            
            # Check if we have LiDAR data
            if self.latest_scan is None:
                return
                
            # Check for collision or near-collision
            is_collision = self.reward_system.check_collision(self.latest_scan)
            is_near_obstacle, obstacle_distance = self.reward_system.check_obstacle_proximity(self.latest_scan)
            
            if is_collision and not self.in_recovery_mode:
                self.get_logger().warn("Collision detected! Entering recovery mode")
                self.in_recovery_mode = True
                self.recovery_count = 0
                # Stop immediately
                self.cmd_vel_pub.publish(Twist())
                return
            
            # Get current robot state
            robot_state = np.array([
                self.robot_position[0],
                self.robot_position[1],
                self.robot_orientation
            ])
            
            # Decide action based on exploration phase
            if self.check_exploration_phase() or not self.is_goal_active:
                linear_vel, angular_vel = self.get_exploration_action()
                self.exploration_steps += 1
            else:
                # Use normal PPO policy for goal-directed movement
                state = self.prepare_state_input()
                with torch.no_grad():
                    action, _ = self.ppo_model.get_action(state, deterministic=True)
                linear_vel = float(action[0, 0].cpu())
                angular_vel = float(action[0, 1].cpu())
            
            # Calculate reward
            action = np.array([linear_vel, angular_vel])
            reward = self.reward_system.get_reward(robot_state, self.latest_scan, action)
            
            # Enhanced debug logging
            if self.debug_mode:
                status = "RECOVERY" if self.in_recovery_mode else ("EXPLORATION" if self.exploration_phase else "NORMAL")
                self.get_logger().debug(
                    f"Step {self.exploration_steps}, Status: {status}, "
                    f"Reward: {reward:.2f}, "
                    f"Obstacle Distance: {obstacle_distance:.2f}"
                )
            
            # Create and publish command
            cmd_vel = Twist()
            cmd_vel.linear.x = linear_vel
            cmd_vel.angular.z = angular_vel
            self.cmd_vel_pub.publish(cmd_vel)
            
            # Publish metrics every second
            if self.exploration_steps % 10 == 0:  # Assuming 10Hz control loop
                self.publish_metrics()
            
            # Check if goal is reached (if in normal operation)
            if not self.exploration_phase and self.is_goal_active:
                self.check_goal_reached()
            
        except Exception as e:
            self.get_logger().error(f"Error in control loop: {str(e)}")
            # Stop the robot in case of error
            self.cmd_vel_pub.publish(Twist())

def main(args=None):
    rclpy.init(args=args)
    node = OperatorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f"Node error: {str(e)}")
    finally:
        # Ensure robot stops when node shuts down
        node.cmd_vel_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
