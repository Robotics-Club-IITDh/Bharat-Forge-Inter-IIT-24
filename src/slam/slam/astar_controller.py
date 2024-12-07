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
        
        # Map quality parameters
        self.min_map_coverage = 0.1  # Minimum fraction of known cells required
        self.map_ready = False  # Flag to indicate if map is ready for navigation
        self.unknown_cell_threshold = -1  # Value in OccupancyGrid for unknown cells
        self.target_area_radius = 1.0  # meters - radius around target to check for mapping
        
        # Robot control parameters - Tuned for smoother movement
        self.linear_speed = 0.3  # m/s
        self.angular_speed = 0.5  # rad/s
        self.position_tolerance = 0.2  # meters
        self.angle_tolerance = 0.15  # radians
        
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
        
    def check_map_quality(self):
        """Check if the map has sufficient coverage for navigation"""
        if not self.map_data:
            return False
            
        # Count known cells
        total_cells = len(self.map_data.data)
        known_cells = sum(1 for cell in self.map_data.data if cell != self.unknown_cell_threshold)
        coverage = known_cells / total_cells
        
        self.get_logger().info(f'Map coverage: {coverage:.2f}')
        return coverage >= self.min_map_coverage
        
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
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )
        
        self.get_logger().info('Set up robot-specific publishers and subscribers')
            
    def target_callback(self, msg):
        """Handle new target position"""
        self.target_position = (msg.x, msg.y)
        self.get_logger().info(f'Received new target: {self.target_position}')
        
        if not self.map_data:
            self.get_logger().warn('No map data available yet')
            return
            
        if not self.check_map_quality():
            self.get_logger().warn('Map quality insufficient for navigation')
            return
            
        if not self.check_target_area_mapped():
            self.get_logger().warn('Target area not sufficiently mapped')
            return
            
        self.get_logger().info('Map ready for navigation, planning path...')
        self.plan_path()
    
    def odom_callback(self, msg):
        """Handle odometry updates"""
        self.current_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )
        orientation = msg.pose.pose.orientation
        _, _, self.current_orientation = euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w]
        )
        
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
        """Reconstruct path from cell details"""
        path = []
        current = goal
        
        while cell_details[current[0]][current[1]].parent_i != current[0] or \
              cell_details[current[0]][current[1]].parent_j != current[1]:
            path.append(self.grid_to_world(current[0], current[1]))
            temp = current
            current = (cell_details[temp[0]][temp[1]].parent_i,
                       cell_details[temp[0]][temp[1]].parent_j)
        
        path.append(self.grid_to_world(current[0], current[1]))
        path.reverse()
        return path
    
    def control_loop(self):
        """Main control loop for robot movement"""
        if not all([self.path, self.current_position, 
                    self.current_path_index < len(self.path), 
                    self.cmd_vel_pub]):
            return
            
        # Get current target point from path
        target = self.path[self.current_path_index]
        
        # Calculate distance and angle to target
        dx = target[0] - self.current_position[0]
        dy = target[1] - self.current_position[1]
        distance = math.sqrt(dx*dx + dy*dy)

        # Check if we're at final target and close enough
        if self.current_path_index == len(self.path) - 1 and distance < self.position_tolerance:
            self.get_logger().info('\n' + '='*50)
            self.get_logger().info('TARGET REACHED SUCCESSFULLY!')
            self.get_logger().info(f'Final position error: {distance:.3f} meters')
            self.get_logger().info('='*50 + '\n')
            self.path = []  # Clear path to stop movement
            
            # Stop the robot
            cmd_vel = Twist()
            self.cmd_vel_pub.publish(cmd_vel)
            return

        target_angle = math.atan2(dy, dx)
        
        # Calculate angle difference
        angle_diff = target_angle - self.current_orientation
        # Normalize angle
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
            
        # Create and publish velocity command
        cmd_vel = Twist()
        
        # Use proportional control for both rotation and forward movement
        if abs(angle_diff) > self.angle_tolerance:
            # Rotate with proportional control
            cmd_vel.angular.z = self.angular_speed * (angle_diff / math.pi)
            # Cap the rotation speed
            cmd_vel.angular.z = max(min(cmd_vel.angular.z, self.angular_speed), -self.angular_speed)
        else:
            # Move forward and make small angle corrections
            if distance > self.position_tolerance:
                # Forward speed proportional to distance, but with a minimum speed
                speed_factor = min(distance, 1.0)
                cmd_vel.linear.x = self.linear_speed * speed_factor
                # Small angle corrections while moving
                cmd_vel.angular.z = 0.3 * angle_diff
            else:
                # Move to next waypoint
                self.current_path_index += 1
                if self.current_path_index >= len(self.path):
                    self.get_logger().info('Completed all waypoints')
                    self.path = []
                else:
                    self.get_logger().info(f'Moving to waypoint {self.current_path_index + 1}/{len(self.path)}')
    
        self.get_logger().debug(f'Distance to target: {distance:.2f}, Angle diff: {math.degrees(angle_diff):.2f} degrees')
        self.get_logger().debug(f'Cmd_vel - linear: {cmd_vel.linear.x:.2f}, angular: {cmd_vel.angular.z:.2f}')
        
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
