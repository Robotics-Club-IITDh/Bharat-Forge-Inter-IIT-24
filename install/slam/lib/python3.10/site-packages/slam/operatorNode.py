#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

class Controller(Node):
    def __init__(self):
        super().__init__("controller")

        # Create QoS profile for laser scanner
        laser_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        
        # Create QoS profile for twist messages
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

        # Publishers
        self.twist_pub_ = self.create_publisher(
            Twist, 
            "/diff_drive_controller/cmd_vel_unstamped", 
            twist_qos
        )
        
        self.get_logger().info("Controller Node Started Successfully")

    def incoming(self, msg: Twist):
        # Forward the twist message to the controller
        self.twist_pub_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()