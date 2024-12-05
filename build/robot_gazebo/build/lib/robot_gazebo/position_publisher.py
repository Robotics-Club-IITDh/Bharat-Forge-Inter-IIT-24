#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import json
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class RobotPositionPublisher(Node):
    def __init__(self):
        super().__init__('robot_position_publisher')
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.declare_parameter('robot_names', ['robot_1', 'robot_2'])
        self.robot_names = self.get_parameter('robot_names').get_parameter_value().string_array_value
        
        self.robot_positions = {}
        
        self.position_publisher = self.create_publisher(
            String,
            '/robot_positions',
            qos_profile
        )

        for robot_name in self.robot_names:
            self.robot_positions[robot_name] = None
            self.create_subscription(
                Odometry,
                f'/{robot_name}/odom',
                self.create_odom_callback(robot_name),
                qos_profile
            )
        
        self.create_timer(0.02, self.publish_positions)

    def create_odom_callback(self, robot_name: str):
        def callback(msg: Odometry):
            self.robot_positions[robot_name] = {
                'x': round(msg.pose.pose.position.x, 3),
                'y': round(msg.pose.pose.position.y, 3),
                'z': round(msg.pose.pose.position.z, 3),
                'qx': round(msg.pose.pose.orientation.x, 3),
                'qy': round(msg.pose.pose.orientation.y, 3),
                'qz': round(msg.pose.pose.orientation.z, 3),
                'qw': round(msg.pose.pose.orientation.w, 3)
            }
        return callback

    def publish_positions(self):
        """Publish all robot positions"""
        if not any(pos is not None for pos in self.robot_positions.values()):
            return
            
        # Compact format for better readability
        output = {
            'timestamp': self.get_clock().now().to_msg().sec,
        }
        
        # Add each robot's data in a more compact format
        for robot_name, pos in self.robot_positions.items():
            if pos is not None:
                output[robot_name] = pos
        
        msg = String()
        msg.data = json.dumps(output, separators=(',', ':'))  # Most compact format
        self.position_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RobotPositionPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()