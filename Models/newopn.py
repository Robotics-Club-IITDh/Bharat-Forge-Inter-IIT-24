import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import torch
import torch.nn as nn
import numpy as np
import os

# Actor Network
class Actor(nn.Module):
    def __init__(self, state_dim, action_dim, max_action):
        super(Actor, self).__init__()
        self.network = nn.Sequential(
            nn.Linear(state_dim, 256),
            nn.ReLU(),
            nn.Linear(256, 256),
            nn.ReLU(),
            nn.Linear(256, action_dim),
            nn.Tanh()
        )
        self.max_action = max_action

    def forward(self, x):
        return self.max_action * self.network(x)

class OperatorNode(Node):
    def __init__(self):
        super().__init__('operator_node')
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        # Actor Model
        actor_model_path = "actor_model.pt"
        self.state_dim = 6  # State dimension matches training
        self.action_dim = 2
        self.max_action = 1.0

        self.actor = Actor(self.state_dim, self.action_dim, self.max_action).to(self.device)
        if os.path.exists(actor_model_path):
            self.actor.load_state_dict(torch.load(actor_model_path, map_location=self.device))
            self.get_logger().info(f"Actor model loaded from {actor_model_path}")
        else:
            self.get_logger().warn(f"Actor model file {actor_model_path} not found.")
        self.actor.eval()

        # Current Pose
        self.current_pose = None

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ROS2 Subscribers and Publishers
        self.odom_sub_ = self.create_subscription(Odometry, '/odom', self.odom_callback, qos)
        self.target_pose_sub_ = self.create_subscription(PoseStamped, '/target_pose', self.target_pose_callback, qos)
        self.twist_pub_ = self.create_publisher(Twist, '/cmd_vel', qos)

        self.get_logger().info("Operator Node Initialized")

    def odom_callback(self, msg: Odometry):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        self.current_pose = {
            'position': np.array([position.x, position.y]),
            'orientation': np.array([orientation.x, orientation.y, orientation.z, orientation.w])
        }

    def target_pose_callback(self, msg: PoseStamped):
        if not self.current_pose:
            self.get_logger().warn("Current pose not available yet.")
            return

        target_position = np.array([msg.pose.position.x, msg.pose.position.y])
        delta_position = target_position - self.current_pose['position']
        state = np.concatenate([delta_position, self.current_pose['orientation']])

        # Get action from actor
        action = self.get_action(state)
        twist_msg = Twist()
        twist_msg.linear.x = action[0] * 0.5  # Scale for linear velocity
        twist_msg.angular.z = action[1] * 2.0  # Scale for angular velocity
        self.twist_pub_.publish(twist_msg)

    def get_action(self, state):
        with torch.no_grad():
            state = torch.FloatTensor(state).to(self.device).unsqueeze(0)
            action = self.actor(state).cpu().numpy().flatten()
        return action

def main(args=None):
    rclpy.init(args=args)
    operator_node = OperatorNode()
    rclpy.spin(operator_node)
    operator_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
