#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile
import math

class PytorchModelController(Node):
    def __init__(self):
        super().__init__("controller")
        
        # Declare and get the namespace parameter
        self.declare_parameter("namespace", "")
        self.namespace = self.get_parameter("namespace").get_parameter_value().string_value
        
        # QoS for reliable communication
        qos_profile = QoSProfile(depth=10)
        
        # Subscribe to the /merged_map topic
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            "/merged_map",
            self.map_callback,
            qos_profile,
        )
        
        # Subscribe to the /{namespace}/odom topic
        self.odom_subscription = self.create_subscription(
            Odometry,
            f"/{self.namespace}/odom",
            self.odom_callback,
            qos_profile,
        )
        
        # Publisher for velocity commands
        self.velocity_publisher = self.create_publisher(
            Twist,
            f"/{self.namespace}/cmd_vel",
            qos_profile,
        )
        
    # /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// 
    # Use these variables for working
    
        # Timer for publishing velocity commands at 20 Hz
        self.timer = self.create_timer(0.05, self.publish_velocity_commands)  # 1/20 = 0.05 seconds
        
        # Simplified variables for processing data
        # Map data
        self.map_data = []  # List to store the flattened map data
        self.map_resolution = 0.0  # Map resolution
        self.map_origin_x = 0.0  # Map origin X-coordinate
        self.map_origin_y = 0.0  # Map origin Y-coordinate
        
        # Odometry data
        self.odom_x = 0.0  # Robot's X position
        self.odom_y = 0.0  # Robot's Y position
        self.odom_theta = 0.0  # Robot's orientation in radians
        self.linear_velocity = 0.0  # Robot's linear velocity
        self.angular_velocity = 0.0  # Robot's angular velocity
        
        # Velocity command variables (to be assigned by the PyTorch model)
        self.linear_cmd = 0.0  # Linear velocity command
        self.angular_cmd = 0.0  # Angular velocity command
        
        self.get_logger().info("Robot Controller Node Started Successfully")

    # ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ############ ROS FUNCTIONS ###################
    def map_callback(self, msg: OccupancyGrid):
        """Callback to handle the incoming /merged_map data."""
        self.map_data = list(msg.data)  # Convert map data to a flat list
        self.map_resolution = msg.info.resolution
        self.map_origin_x = msg.info.origin.position.x
        self.map_origin_y = msg.info.origin.position.y
        self.get_logger().debug("Map data updated.")
    
    def odom_callback(self, msg: Odometry):
        """Callback to handle the incoming odometry data."""
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        # Convert quaternion to yaw (theta)
        self.odom_theta = self.quaternion_to_yaw(
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        )
        self.linear_velocity = msg.twist.twist.linear.x
        self.angular_velocity = msg.twist.twist.angular.z
        self.get_logger().debug("Odometry data updated.")
    
    def publish_velocity_commands(self):
        """Publishes velocity commands to the /cmd_vel topic."""
        cmd_msg = Twist()
        cmd_msg.linear.x = self.linear_cmd
        cmd_msg.angular.z = self.angular_cmd
        self.velocity_publisher.publish(cmd_msg)
        self.get_logger().debug("Published velocity commands: Linear: %.2f, Angular: %.2f" % (self.linear_cmd, self.angular_cmd))
    
    @staticmethod
    def quaternion_to_yaw(x, y, z, w):
        """Convert a quaternion into a yaw angle in radians."""
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

def main(args=None):
    rclpy.init(args=args)
    node = PytorchModelController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
