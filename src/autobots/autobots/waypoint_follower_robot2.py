#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import math

class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower_robot2')
        
        # Waypoints in world coordinates
        self.waypoints = [
            (5.501, -5.816),    # First waypoint (spawn point)
            (0.752, -11.426),   # Second waypoint
            (5.501, -5.816)     # Back to start
        ]
        
        # Store spawn point (first waypoint) for offset calculations
        self.spawn_x = self.waypoints[0][0]
        self.spawn_y = self.waypoints[0][1]
        
        self.current_waypoint = 0
        self.x = None
        self.y = None
        self.theta = None
        self.initialized = False
        
        # Control parameters
        self.linear_speed = 0.3
        self.angular_speed = 0.5
        self.goal_tolerance = 0.2
        
        # Publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/robot_2/cmd_vel',  # Updated topic for robot_2
            10
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/robot_2/odom',    # Updated topic for robot_2
            self.odom_callback,
            10
        )
        
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('Robot 2 waypoint follower initialized')
        self.get_logger().info(f'Spawn point: ({self.spawn_x}, {self.spawn_y})')
    def transform_to_world(self, odom_x, odom_y):
        """Transform odometry coordinates to world coordinates"""
        # Since robot spawns at first waypoint but odometry reads (0,0)
        # we need to add the spawn point offset
        world_x = odom_x + self.spawn_x
        world_y = odom_y + self.spawn_y
        return world_x, world_y

    def odom_callback(self, msg):
        # Get raw odometry
        odom_x = msg.pose.pose.position.x
        odom_y = msg.pose.pose.position.y
        
        # Transform to world coordinates
        self.x, self.y = self.transform_to_world(odom_x, odom_y)
        
        orientation = msg.pose.pose.orientation
        _, _, self.theta = euler_from_quaternion([
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w
        ])

        if not self.initialized:
            self.get_logger().info(f'Odometry position: ({odom_x:.3f}, {odom_y:.3f})')
            self.get_logger().info(f'Transformed world position: ({self.x:.3f}, {self.y:.3f})')
            self.initialized = True

    def get_distance_to_goal(self):
        if not self.initialized:
            return float('inf')
        goal_x, goal_y = self.waypoints[self.current_waypoint]
        return math.sqrt(pow(goal_x - self.x, 2) + pow(goal_y - self.y, 2))

    def get_angle_to_goal(self):
        if not self.initialized:
            return 0.0
        goal_x, goal_y = self.waypoints[self.current_waypoint]
        angle = math.atan2(goal_y - self.y, goal_x - self.x)
        return self.normalize_angle(angle - self.theta)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def control_loop(self):
        if not self.initialized:
            return
            
        distance = self.get_distance_to_goal()
        angle_to_goal = self.get_angle_to_goal()
        
        # Log current status periodically
        if self.get_clock().now().nanoseconds % 1000000000 < 100000000:
            goal_x, goal_y = self.waypoints[self.current_waypoint]
            self.get_logger().info(f'Current world position: ({self.x:.3f}, {self.y:.3f})')
            self.get_logger().info(f'Target waypoint: ({goal_x:.3f}, {goal_y:.3f})')
            self.get_logger().info(f'Distance: {distance:.3f}, Angle: {math.degrees(angle_to_goal):.1f}°')
        
        # Check if reached waypoint
        if distance < self.goal_tolerance:
            self.current_waypoint = (self.current_waypoint + 1) % len(self.waypoints)
            next_wp = self.waypoints[self.current_waypoint]
            self.get_logger().info(f'Reached waypoint, moving to: ({next_wp[0]:.3f}, {next_wp[1]:.3f})')
            return
        
        cmd = Twist()
        
        # If not facing the goal, turn first
        if abs(angle_to_goal) > 0.1:
            cmd.angular.z = self.angular_speed if angle_to_goal > 0 else -self.angular_speed
            cmd.linear.x = 0.0
        else:
            # Move towards goal with proportional control
            cmd.linear.x = min(self.linear_speed, distance)
            cmd.angular.z = 0.3 * angle_to_goal
        
        self.cmd_vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollower()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()   