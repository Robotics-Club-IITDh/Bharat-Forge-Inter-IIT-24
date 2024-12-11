#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
from collections import deque
import copy
import math
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

class DynamicLaserFilter(Node):
    def __init__(self):
        super().__init__('dynamic_laser_filter')
        
        # Parameters
        self.declare_parameter('scan_topic', 'scan')
        self.declare_parameter('buffer_size', 5)
        self.declare_parameter('distance_threshold', 0.3)
        self.declare_parameter('persistence_threshold', 3)
        self.declare_parameter('min_static_readings', 4)
        
        # Configure QoS profiles
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        
        # Point buffers for tracking
        self.point_buffers = {}
        
        # Subscribe to raw scan
        self.scan_sub = self.create_subscription(
            LaserScan,
            self.get_parameter('scan_topic').value,
            self.scan_callback,
            sensor_qos
        )
        
        # Publisher for filtered scan
        self.scan_pub = self.create_publisher(
            LaserScan,
            'filtered_scan',
            sensor_qos
        )
        
        self.get_logger().info('Dynamic Laser Filter initialized')

    def process_scan_in_base_frame(self, scan_msg):
        """Convert scan ranges to Cartesian coordinates in base frame."""
        points = []
        angle = scan_msg.angle_min
        
        for range_val in scan_msg.ranges:
            if scan_msg.range_min <= range_val <= scan_msg.range_max:
                x = range_val * math.cos(angle)
                y = range_val * math.sin(angle)
                points.append((x, y))
            else:
                points.append(None)
            angle += scan_msg.angle_increment
            
        return points

    def is_point_dynamic(self, current_point, angle_idx):
        """Determine if a point is dynamic based on its movement history."""
        if current_point is None:
            return False
            
        if angle_idx not in self.point_buffers:
            self.point_buffers[angle_idx] = deque(maxlen=self.get_parameter('buffer_size').value)
            
        buffer = self.point_buffers[angle_idx]
        buffer.append(current_point)
        
        if len(buffer) < 2:
            return False
            
        distance_threshold = self.get_parameter('distance_threshold').value
        persistence_threshold = self.get_parameter('persistence_threshold').value
        
        dynamic_count = 0
        
        for prev_point in list(buffer)[:-1]:
            if prev_point is None:
                continue
                
            dx = current_point[0] - prev_point[0]
            dy = current_point[1] - prev_point[1]
            distance = math.sqrt(dx*dx + dy*dy)
            
            if distance > distance_threshold:
                dynamic_count += 1
                
        return dynamic_count >= persistence_threshold

    def scan_callback(self, msg: LaserScan):
        """Process incoming LaserScan messages."""
        if len(msg.ranges) == 0:
            self.get_logger().warning('Received empty scan!')
            return
            
        # Create filtered scan with exact copy of metadata
        filtered_scan = LaserScan()
        filtered_scan.header = msg.header  # Preserve original timestamp and frame_id
        filtered_scan.angle_min = msg.angle_min
        filtered_scan.angle_max = msg.angle_max
        filtered_scan.angle_increment = msg.angle_increment
        filtered_scan.time_increment = msg.time_increment
        filtered_scan.scan_time = msg.scan_time
        filtered_scan.range_min = msg.range_min
        filtered_scan.range_max = msg.range_max
        filtered_scan.intensities = msg.intensities if hasattr(msg, 'intensities') else []
        
        # Process points
        base_frame_points = self.process_scan_in_base_frame(msg)
        filtered_ranges = list(msg.ranges)
        
        # Filter dynamic points
        for i, point in enumerate(base_frame_points):
            if point is not None and self.is_point_dynamic(point, i):
                filtered_ranges[i] = msg.range_max
        
        filtered_scan.ranges = tuple(filtered_ranges)
        
        # Update header timestamp to current time if using sim time
        if self.get_parameter('use_sim_time').value:
            filtered_scan.header.stamp = self.get_clock().now().to_msg()
        
        # Debug info
        self.get_logger().debug(
            f'Publishing filtered scan with {len(filtered_ranges)} points, '
            f'frame_id: {filtered_scan.header.frame_id}, '
            f'stamp: {filtered_scan.header.stamp.sec}.{filtered_scan.header.stamp.nanosec}'
        )
        
        # Publish filtered scan
        self.scan_pub.publish(filtered_scan)

def main(args=None):
    rclpy.init(args=args)
    node = DynamicLaserFilter()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()