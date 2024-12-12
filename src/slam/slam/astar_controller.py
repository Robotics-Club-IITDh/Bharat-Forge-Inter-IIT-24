#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import String
import numpy as np
import math
import heapq
from tf_transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan 

class Cell:
    def __init__(self):
        self.parent_i = 0
        self.parent_j = 0
        self.f = float('inf')
        self.g = float('inf')
        self.h = 0

class AStarController(Node):
    def __init__(self):
        super().__init__('astar_controller')

        self.waypoint_spacing = 1.0  # meters between waypoints
        self.path_smoothing_factor = 0.25  # controls how much to smooth the path
        self.min_corner_angle = math.pi/6  # minimum angle to consider as a corner

        # Add flag for tracking subscription status
        self.odom_received = False
        self.pending_target = None
        
        # Declare a parameter for the robot namespace (passed in at launch)
        self.robot_namespace = self.declare_parameter('robot_namespace', '').value
        
        # Robot configuration
        self.robot_width = 0.3  # meters
        self.robot_length = 0.4  # meters
        self.safety_margin = 0.2  # meters
        
        # Navigation parameters
        self.target_position = None
        self.current_position = None
        self.current_orientation = 0.0
        self.map_data = None
        self.robot_name = None
        self.path = []
        self.current_path_index = 0
        
        # Obstacle detection parameters
        self.emergency_stop_distance = 0.4  # meters
        self.replanning_cooldown = 2.0  # seconds
        self.last_replan_time = 0.0
        self.latest_scan = None
        self.min_lidar_distance = 0.6  # minimum safe distance from obstacles
        self.scan_angle_window = math.pi  # Check ±45 degrees in front of robot

        # Map quality parameters
        self.min_map_coverage = 0.1  # Minimum fraction of known cells required
        self.map_ready = False  # Flag to indicate if map is ready for navigation
        self.unknown_cell_threshold = -1  # Value in OccupancyGrid for unknown cells
        self.target_area_radius = 0.1  # meters - radius around target to check for mapping
        
        # Robot control parameters - Tuned for smoother movement
        self.linear_speed = 0.5  # m/s
        self.angular_speed = 0.5  # rad/s
        self.position_tolerance = 0.1  # meters
        self.angle_tolerance = 0.15  # radians
        

        self.last_replan_time = None
        self.replanning_cooldown = 1.0  # or whatever value you want for cooldown in seconds

        # Create global subscribers
        # Global map topic
        self.create_subscription(
            OccupancyGrid,
            '/merge_map',  # Global merged map topic
            self.map_callback,
            10
        )
        
        self.create_subscription(
            String,
            '/target_return',
            self.target_return_callback,
            10
        )
        
        self.create_subscription(
            Point,
            '/target',
            self.target_callback,
            10
        )
        
        # Publisher for robot velocity commands will be created after receiving robot name
        self.cmd_vel_pub = None
        
        # Control loop timer
        self.create_timer(0.1, self.control_loop)
        self.current_coverage = 0.0
        self.create_timer(5.0, self.log_map_coverage)

    def check_map_quality(self):
        """Check if the map has sufficient coverage for navigation"""
        if not self.map_data:
            return False
            
        # Count known cells
        total_cells = len(self.map_data.data)
        known_cells = sum(1 for cell in self.map_data.data if cell != self.unknown_cell_threshold)
        self.current_coverage = known_cells / total_cells
        
        return self.current_coverage >= self.min_map_coverage

    def log_map_coverage(self):
        """Separate timer callback to log map coverage at 0.5 Hz"""
        if self.map_data:  # Only log if we have map data
            self.get_logger().info(f'Map coverage: {self.current_coverage:.2f}')
        
    def check_target_area_mapped(self):
        """Check if the area around the target is sufficiently mapped"""
        if not all([self.map_data, self.target_position]):
            return False
            
        target_grid = self.world_to_grid(*self.target_position)
        if not target_grid:
            return False
            
        # Calculate grid radius
        radius_cells = int(self.target_area_radius / self.map_data.info.resolution)
        
        # Check cells in a square around target
        for i in range(-radius_cells, radius_cells + 1):
            for j in range(-radius_cells, radius_cells + 1):
                check_x = target_grid[0] + i
                check_y = target_grid[1] + j
                
                if not (0 <= check_x < self.map_data.info.width and 
                       0 <= check_y < self.map_data.info.height):
                    continue
                    
                idx = check_y * self.map_data.info.width + check_x
                if self.map_data.data[idx] == self.unknown_cell_threshold:
                    return False
                    
        return True
        
    def target_return_callback(self, msg):
        """Handle robot selection message"""
        self.robot_name = msg.data
        self.get_logger().info(f'Received robot selection: {self.robot_name}')
        
        # Once we have the robot_name, set up publishers and subscribers for that robot
        # Because the node is launched in the controller namespace (robot_namespace), we can use relative topics.
        # This will resolve to /<robot_namespace>/cmd_vel and /<robot_namespace>/odom
        # Fix: Use fully qualified topic name with namespace
        odom_topic = f'/{self.robot_name}/odom'
        self.get_logger().info(f'Subscribing to odometry topic: {odom_topic}')
        
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry,
            odom_topic,  # Now using the full topic name
            self.odom_callback,
            10
        )

        scan_topic = f'/{self.robot_name}/scan'
        self.scan_sub = self.create_subscription(
            LaserScan,
            scan_topic,
            self.scan_callback,
            10
        )
        self.get_logger().info(f'Subscribing to scan topic: {scan_topic}')
        
        self.get_logger().info('Set up robot-specific publishers and subscribers')
    
    def scan_callback(self, msg):
        """Handle new LiDAR scan data"""
        self.latest_scan = msg


    def odom_callback(self, msg):
        """Handle odometry updates"""
        # Extract position
        self.current_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )
        
        # Extract orientation (convert quaternion to Euler angles)
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, yaw = euler_from_quaternion(orientation_list)
        self.current_orientation = yaw

        # If this is the first odometry message received
        if not self.odom_received:
            self.odom_received = True
            self.get_logger().info('First odometry message received')
            
            # If there was a pending target, process it now
            if self.pending_target:
                self.get_logger().info('Processing pending target')
                target_msg = Point()
                target_msg.x = self.pending_target[0]
                target_msg.y = self.pending_target[1]
                self.target_callback(target_msg)
                self.pending_target = None

    def target_callback(self, msg):
        """Handle new target position"""
        # Check if odometry subscription is ready
        if not hasattr(self, 'odom_sub'):
            self.get_logger().warn('Received target but odometry subscription not yet setup. Waiting for robot selection...')
            # Store target for later use
            self.pending_target = (msg.x, msg.y)
            return
                
        self.target_position = (msg.x, msg.y)
        self.get_logger().info(f'Received new target: {self.target_position}')
        
        # Add detailed logging for each check
        if not self.current_position:
            self.get_logger().warn('No odometry data received yet. Will plan path once odometry is available.')
            return

        if not self.map_data:
            self.get_logger().warn('No map data available yet')
            return

        # Print current map quality values
        self.get_logger().info(f'Current map coverage: {self.current_coverage:.2f}, Required: {self.min_map_coverage}')
        if not self.check_map_quality():
            self.get_logger().warn(f'Map quality insufficient for navigation. Coverage: {self.current_coverage:.2f}')
            return
                
        if not self.check_target_area_mapped():
            self.get_logger().warn('Target area not sufficiently mapped')
            self.get_logger().info(f'Target position in grid coordinates: {self.world_to_grid(*self.target_position)}')
            return

        self.get_logger().info('All checks passed, planning path...')
        self.get_logger().info(f'Planning path from {self.current_position} to {self.target_position}')
        self.plan_path()

        # Log path planning result
        if self.path:
            self.get_logger().info(f'Successfully planned path with {len(self.path)} waypoints')
        else:
            self.get_logger().warn('Failed to plan a path to target')
        
    def map_callback(self, msg):
        """Handle map updates"""
        self.map_data = msg
        map_was_ready = self.map_ready
        self.map_ready = self.check_map_quality()
        
        if self.map_ready and not map_was_ready and self.target_position:
            self.get_logger().info('Map newly ready for navigation, planning path...')
            self.plan_path()
    
    def world_to_grid(self, world_x, world_y):
        """Convert world coordinates to grid coordinates"""
        if not self.map_data:
            return None
        
        grid_x = int((world_x - self.map_data.info.origin.position.x) / 
                     self.map_data.info.resolution)
        grid_y = int((world_y - self.map_data.info.origin.position.y) / 
                     self.map_data.info.resolution)
        
        if 0 <= grid_x < self.map_data.info.width and \
           0 <= grid_y < self.map_data.info.height:
            return (grid_x, grid_y)
        return None
    
    def grid_to_world(self, grid_x, grid_y):
        """Convert grid coordinates to world coordinates"""
        world_x = grid_x * self.map_data.info.resolution + \
                  self.map_data.info.origin.position.x
        world_y = grid_y * self.map_data.info.resolution + \
                  self.map_data.info.origin.position.y
        return (world_x, world_y)
    
    def is_valid_cell(self, x, y):
        """Check if cell is within map bounds and not occupied"""
        if not (0 <= x < self.map_data.info.width and 
                0 <= y < self.map_data.info.height):
            return False
            
        # Check cell and surrounding cells for obstacles
        margin_cells = int(self.safety_margin / self.map_data.info.resolution)
        for i in range(-margin_cells, margin_cells + 1):
            for j in range(-margin_cells, margin_cells + 1):
                check_x = x + i
                check_y = y + j
                if (0 <= check_x < self.map_data.info.width and 
                    0 <= check_y < self.map_data.info.height):
                    idx = check_y * self.map_data.info.width + check_x
                    if self.map_data.data[idx] > 50:  # Occupied cell
                        return False
        return True
    
    def calculate_h_value(self, x1, y1, x2, y2):
        """Calculate heuristic value (Euclidean distance)"""
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
    
    def check_for_obstacles(self):
        """Check for obstacles using LiDAR scan data"""
        if not self.latest_scan:
            self.get_logger().warn('No LiDAR scan data available - stopping for safety')
            return True

        # For full 360 degrees, check all indices
        for i in range(len(self.latest_scan.ranges)):
            range_val = self.latest_scan.ranges[i]
            
            # Skip invalid measurements
            if math.isnan(range_val) or math.isinf(range_val):
                continue
            if range_val < self.latest_scan.range_min:
                self.get_logger().warn(f'Very close obstacle detected!')
                return True
            if range_val > self.latest_scan.range_max:
                continue
                
            # If any measurement is closer than minimum safe distance
            if range_val < self.min_lidar_distance:
                angle = self.latest_scan.angle_min + (i * self.latest_scan.angle_increment)
                self.get_logger().warn(f'Obstacle detected at {range_val:.2f}m, angle: {math.degrees(angle):.1f}°')
                return True
                
        return False

    def is_near_obstacle(self, point):
        """Check if a point is near any obstacles"""
        grid_point = self.world_to_grid(*point)
        if not grid_point:
            return False
            
        check_radius = int(self.safety_margin / self.map_data.info.resolution)
        
        for dx in range(-check_radius, check_radius + 1):
            for dy in range(-check_radius, check_radius + 1):
                check_x = grid_point[0] + dx
                check_y = grid_point[1] + dy
                
                if not (0 <= check_x < self.map_data.info.width and 
                    0 <= check_y < self.map_data.info.height):
                    continue
                    
                idx = check_y * self.map_data.info.width + check_x
                if self.map_data.data[idx] > 50:  # Obstacle detected
                    return True
        return False

    def reduce_waypoints(self, raw_path):
        """Reduce the number of waypoints while maintaining path integrity"""
        if len(raw_path) < 3:
            return raw_path
            
        reduced_path = [raw_path[0]]  # Always keep start point
        current_distance = 0
        last_significant_angle = 0
        
        for i in range(1, len(raw_path) - 1):
            # Calculate angles between consecutive segments
            prev_vector = (
                raw_path[i][0] - raw_path[i-1][0],
                raw_path[i][1] - raw_path[i-1][1]
            )
            next_vector = (
                raw_path[i+1][0] - raw_path[i][0],
                raw_path[i+1][1] - raw_path[i][1]
            )
            
            # Calculate angle between vectors
            angle = math.atan2(
                prev_vector[0] * next_vector[1] - prev_vector[1] * next_vector[0],
                prev_vector[0] * next_vector[0] + prev_vector[1] * next_vector[1]
            )
            
            # Calculate distance from last waypoint
            dist_from_last = math.sqrt(
                (raw_path[i][0] - reduced_path[-1][0])**2 +
                (raw_path[i][1] - reduced_path[-1][1])**2
            )
            
            # Add waypoint if:
            # 1. We've exceeded the minimum spacing, or
            # 2. There's a significant turn, or
            # 3. The point is near an obstacle
            if (dist_from_last >= self.waypoint_spacing or
                abs(angle) >= self.min_corner_angle or
                self.is_near_obstacle(raw_path[i])):
                reduced_path.append(raw_path[i])
                current_distance = 0
            
        reduced_path.append(raw_path[-1])  # Always keep end point
        return self.smooth_path(reduced_path)

    def smooth_path(self, path):
        """Apply path smoothing to create more natural trajectories"""
        if len(path) < 3:
            return path
            
        smoothed = list(path)  # Create a copy to modify
        
        for _ in range(5):  # Number of smoothing iterations
            for i in range(1, len(smoothed) - 1):
                # Calculate new position
                smoothed[i] = (
                    smoothed[i][0] + self.path_smoothing_factor * (
                        0.5 * (smoothed[i-1][0] + smoothed[i+1][0]) - smoothed[i][0]
                    ),
                    smoothed[i][1] + self.path_smoothing_factor * (
                        0.5 * (smoothed[i-1][1] + smoothed[i+1][1]) - smoothed[i][1]
                    )
                )
                
                # Ensure smoothed point doesn't create collision
                if self.is_near_obstacle(smoothed[i]):
                    smoothed[i] = path[i]  # Revert to original point
                    
        return smoothed

    def plan_path(self):
        """Plan path using A* algorithm"""
        if not all([self.map_data, self.current_position, self.target_position]):
            self.get_logger().warn('Missing data for path planning')
            self.get_logger().info(f'Map data: {bool(self.map_data)}, Current pos: {bool(self.current_position)}, Target pos: {bool(self.target_position)}')
            return
            
        # Convert positions to grid coordinates
        start_grid = self.world_to_grid(*self.current_position)
        goal_grid = self.world_to_grid(*self.target_position)
        
        if not start_grid or not goal_grid:
            self.get_logger().warn('Start or goal position outside map bounds')
            return
            
        # Initialize A* algorithm
        closed_list = set()
        open_list = []
        cell_details = [[Cell() for _ in range(self.map_data.info.height)]
                        for _ in range(self.map_data.info.width)]
        
        # Add starting position to open list
        start_cell = cell_details[start_grid[0]][start_grid[1]]
        start_cell.f = 0
        start_cell.g = 0
        start_cell.h = 0
        start_cell.parent_i = start_grid[0]
        start_cell.parent_j = start_grid[1]
        heapq.heappush(open_list, (0, start_grid[0], start_grid[1]))
        
        while open_list:
            current = heapq.heappop(open_list)
            current_i, current_j = current[1], current[2]
            
            if (current_i, current_j) == goal_grid:
                # Path found, reconstruct it
                self.path = self.reconstruct_path(cell_details, goal_grid)
                self.current_path_index = 0
                self.get_logger().info(f'Path found with {len(self.path)} waypoints')
                return
                
            closed_list.add((current_i, current_j))
            
            # Check all neighbors
            for di, dj in [(0,1), (1,0), (0,-1), (-1,0), 
                           (1,1), (1,-1), (-1,1), (-1,-1)]:
                neighbor_i = current_i + di
                neighbor_j = current_j + dj
                
                if not self.is_valid_cell(neighbor_i, neighbor_j):
                    continue
                    
                if (neighbor_i, neighbor_j) in closed_list:
                    continue
                
                # Calculate new path cost
                step_cost = 1.414 if di * dj != 0 else 1.0
                new_g = cell_details[current_i][current_j].g + step_cost
                
                neighbor_cell = cell_details[neighbor_i][neighbor_j]
                if neighbor_cell.f == float('inf') or neighbor_cell.g > new_g:
                    h = self.calculate_h_value(neighbor_i, neighbor_j, 
                                               goal_grid[0], goal_grid[1])
                    neighbor_cell.f = new_g + h
                    neighbor_cell.g = new_g
                    neighbor_cell.h = h
                    neighbor_cell.parent_i = current_i
                    neighbor_cell.parent_j = current_j
                    heapq.heappush(open_list, 
                                   (neighbor_cell.f, neighbor_i, neighbor_j))
        
        self.get_logger().warn('No path found to target')
        
    def reconstruct_path(self, cell_details, goal):
        """Reconstruct path from cell details with reduced waypoints"""
        # First get the complete path
        raw_path = []
        current = goal
        
        while cell_details[current[0]][current[1]].parent_i != current[0] or \
            cell_details[current[0]][current[1]].parent_j != current[1]:
            raw_path.append(self.grid_to_world(current[0], current[1]))
            temp = current
            current = (cell_details[temp[0]][temp[1]].parent_i,
                    cell_details[temp[0]][temp[1]].parent_j)
        
        raw_path.append(self.grid_to_world(current[0], current[1]))
        raw_path.reverse()
        
        # Now reduce the waypoints
        return self.reduce_waypoints(raw_path)
    
    def get_front_sector_ranges(self, scan: LaserScan, angle_range: float = math.pi/4):
        """
        Extract ranges from the front sector of the LiDAR scan
        """
        # Convert angle to index
        start_index = int((scan.angle_min + math.pi/2 - angle_range/2 - scan.angle_min) / scan.angle_increment)
        end_index = int((scan.angle_min + math.pi/2 + angle_range/2 - scan.angle_min) / scan.angle_increment)
        
        # Filter out inf and nan values
        front_ranges = [
            r for r in scan.ranges[start_index:end_index] 
            if r != float('inf') and not math.isnan(r)
        ]
        
        return front_ranges

    def avoid_obstacle(self, scan: LaserScan):
        """
        Enhanced obstacle avoidance strategy with dynamic speed adjustment
        """
        avoid_cmd = Twist()
        
        # Get ranges from different sectors
        front_ranges = self.get_front_sector_ranges(scan, angle_range=math.pi/4)
        left_ranges = scan.ranges[:len(scan.ranges)//3]
        right_ranges = scan.ranges[2*len(scan.ranges)//3:]
        
        min_front = min(front_ranges) if front_ranges else float('inf')
        min_left = min([r for r in left_ranges if not math.isnan(r) and not math.isinf(r)], default=float('inf'))
        min_right = min([r for r in right_ranges if not math.isnan(r) and not math.isinf(r)], default=float('inf'))
        
        # Dynamic speed adjustment based on obstacle proximity
        linear_speed = min(0.4, max(0.1, min_front - 0.3))  # Scale speed with distance
        
        # Decision making for avoidance direction
        if min_front < 0.5:  # If obstacle is close in front
            avoid_cmd.linear.x = linear_speed
            
            # Choose rotation direction based on more free space
            if min_left > min_right:
                avoid_cmd.angular.z = 0.8  # Stronger rotation when closer
            else:
                avoid_cmd.angular.z = -0.8
        else:
            # More gentle avoidance when obstacle is further
            avoid_cmd.linear.x = linear_speed
            if min_left > min_right:
                avoid_cmd.angular.z = -0.4
            else:
                avoid_cmd.angular.z = 0.4
        
        return avoid_cmd

    def control_loop(self):
        """Main control loop for robot movement with integrated obstacle avoidance"""
        if not all([self.path, self.current_position, self.cmd_vel_pub]):
            return
            
        if not self.current_path_index < len(self.path):
            return

        # Initialize stop command in case needed
        stop_cmd = Twist()

        # Check for obstacles using LiDAR
        if self.latest_scan and self.check_for_obstacles():
            self.get_logger().warn('Obstacle detected - initiating avoidance maneuver')
            
            # Execute obstacle avoidance
            avoid_cmd = self.avoid_obstacle(self.latest_scan)
            self.cmd_vel_pub.publish(avoid_cmd)
            
            # Check if we should replan
            current_time = self.get_clock().now()
            if self.last_replan_time is None:
                self.last_replan_time = current_time
            else:
                cooldown_duration = rclpy.duration.Duration(seconds=self.replanning_cooldown)
                if (current_time - self.last_replan_time) > cooldown_duration:
                    self.get_logger().info('Replanning path after avoidance maneuver...')
                    self.last_replan_time = current_time
                    self.plan_path()
            return

        # Get current target point from path
        target = self.path[self.current_path_index]
        
        # Calculate distance and angle to target
        dx = target[0] - self.current_position[0]
        dy = target[1] - self.current_position[1]
        distance = math.sqrt(dx*dx + dy*dy)

        # Check if reached final target
        if self.current_path_index == len(self.path) - 1 and distance < self.position_tolerance:
            self.get_logger().info('Target reached successfully!')
            self.path = []
            self.cmd_vel_pub.publish(stop_cmd)
            return

        # Calculate target angle and angle difference
        target_angle = math.atan2(dy, dx)
        angle_diff = target_angle - self.current_orientation
        
        # Normalize angle difference
        while angle_diff > math.pi: angle_diff -= 2 * math.pi
        while angle_diff < -math.pi: angle_diff += 2 * math.pi
            
        # Create velocity command
        cmd_vel = Twist()
        
        # Rotation phase - align with target
        if abs(angle_diff) > self.angle_tolerance:
            cmd_vel.angular.z = self.angular_speed * (angle_diff / math.pi)
            cmd_vel.angular.z = max(min(cmd_vel.angular.z, self.angular_speed), -self.angular_speed)
        
        # Movement phase - proceed to target
        else:
            if distance > self.position_tolerance:
                # Forward speed proportional to distance
                speed_factor = min(distance, 1.0)
                cmd_vel.linear.x = self.linear_speed * speed_factor
                
                # Small angle corrections while moving
                cmd_vel.angular.z = 0.3 * angle_diff
                
            # Reached current waypoint
            else:
                self.current_path_index += 1
                if self.current_path_index >= len(self.path):
                    self.get_logger().info('Completed all waypoints')
                    self.path = []
                return
        
        # Publish velocity command
        self.cmd_vel_pub.publish(cmd_vel)


def main(args=None):
    rclpy.init(args=args)
    node = AStarController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
