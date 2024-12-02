#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import time

class ScanThrottler(Node):
    def __init__(self):
        super().__init__('scan_throttler')
        
        # Parameters
        self.declare_parameter('robot_namespace', '')
        self.declare_parameter('throttle_rate', 5.0)
        
        namespace = self.get_parameter('robot_namespace').get_parameter_value().string_value
        self.target_rate = self.get_parameter('throttle_rate').get_parameter_value().double_value
        
        # Timing management
        self.last_publish_time = self.get_clock().now()
        self.min_interval = 1.0 / self.target_rate
        
        # Enhanced QoS Profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,  # Changed to RELIABLE
            history=HistoryPolicy.KEEP_LAST,
            depth=2,
            durability=DurabilityPolicy.VOLATILE
        )

        self.publisher = self.create_publisher(
            LaserScan,
            'throttled_scan',
            qos_profile
        )

        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile
        )

        # Buffer for transform sync
        self.msg_buffer = None
        self.get_logger().info(f'Scan throttler initialized: {namespace}, Rate: {self.target_rate}Hz')

    def scan_callback(self, msg):
        current_time = self.get_clock().now()
        time_since_last = (current_time - self.last_publish_time).nanoseconds / 1e9
        
        if time_since_last >= self.min_interval:
            # Update timestamp slightly behind current time to ensure transform availability
            publish_time = current_time
            msg.header.stamp = publish_time.to_msg()
            
            try:
                self.publisher.publish(msg)
                self.last_publish_time = current_time
            except Exception as e:
                self.get_logger().error(f'Publish error: {str(e)}')