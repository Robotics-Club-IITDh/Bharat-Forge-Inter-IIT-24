import numpy as np
from scipy.signal import medfilt

class ExplorationReward:
    def __init__(self, num_beams, max_range=3.5, collision_threshold=0.3,
                 collision_penalty=100.0, exploration_reward=50.0, 
                 free_space_reward=20.0, movement_reward=5.0,
                 consistency_reward=10.0):
        """
        Initialize Reward system with exploration focus and collision recovery
        """
        self.num_beams = num_beams
        self.max_range = max_range
        self.collision_threshold = collision_threshold
        self.collision_penalty = collision_penalty
        self.exploration_reward = exploration_reward
        self.free_space_reward = free_space_reward
        self.movement_reward = movement_reward
        self.consistency_reward = consistency_reward
        
        # Keep track of explored areas and path
        self.visited_positions = set()
        self.path_history = []  # Store recent positions
        self.max_path_history = 50  # Number of positions to remember
        self.grid_resolution = 0.5
        
        # Previous state tracking
        self.previous_scan = None
        self.previous_action = None
        self.collision_count = 0
        self.max_collisions = 3  # Maximum consecutive collisions before forcing direction change
        self.recovery_mode = False
        self.recovery_steps = 0
        self.max_recovery_steps = 20
        
        # Direction persistence
        self.current_direction = None
        self.direction_steps = 0
        self.min_direction_steps = 10  # Minimum steps to maintain direction

    def discretize_position(self, position):
        """Convert continuous position to discrete grid cell"""
        x, y = position
        return (round(x / self.grid_resolution) * self.grid_resolution,
                round(y / self.grid_resolution) * self.grid_resolution)

    def update_path_history(self, position):
        """Update the path history with current position"""
        self.path_history.append(position)
        if len(self.path_history) > self.max_path_history:
            self.path_history.pop(0)

    def get_recovery_action(self):
        """Generate recovery action when stuck or after collision"""
        if len(self.path_history) < 2:
            return np.array([-0.2, np.random.uniform(-1.0, 1.0)])  # Back up and turn randomly
            
        # Calculate direction away from last collision
        current_pos = np.array(self.path_history[-1])
        prev_pos = np.array(self.path_history[-2])
        direction = current_pos - prev_pos
        
        # Generate action that moves away from collision
        reverse_direction = -direction / (np.linalg.norm(direction) + 1e-6)
        angular_vel = np.arctan2(reverse_direction[1], reverse_direction[0])
        
        return np.array([-0.2, angular_vel])  # Back up while turning

    def process_lidar(self, scan):
        """Process LiDAR data and detect open spaces and obstacles"""
        if scan is None:
            return 0.0, []
            
        processed_scan = np.array(scan)
        processed_scan[np.isinf(processed_scan)] = self.max_range
        
        # Detect open spaces
        open_spaces = processed_scan > (self.max_range * 0.7)
        open_space_ratio = np.mean(open_spaces)
        
        # Find continuous open space segments
        open_segments = []
        start_idx = None
        
        for i, is_open in enumerate(open_spaces):
            if is_open and start_idx is None:
                start_idx = i
            elif not is_open and start_idx is not None:
                open_segments.append((start_idx, i))
                start_idx = None
                
        if start_idx is not None:
            open_segments.append((start_idx, len(open_spaces)))
        
        return open_space_ratio * self.free_space_reward, open_segments

    def check_obstacle_proximity(self, scan):
        """Check for obstacles in robot's path"""
        if scan is None:
            return False, float('inf')
        
        # Focus on the front sectors of the scan
        front_scan = np.array(scan[len(scan)//3:2*len(scan)//3])
        front_scan = front_scan[~np.isinf(front_scan)]
        
        if len(front_scan) == 0:
            return False, float('inf')
        
        min_distance = np.min(front_scan)
        is_close = min_distance < (self.collision_threshold * 2)  # More conservative threshold
        
        return is_close, min_distance

    def check_collision(self, scan):
        """Check if robot is in collision state"""
        if scan is None:
            return False
            
        scan_array = np.array(scan)
        scan_array = scan_array[~np.isinf(scan_array)]
        
        if len(scan_array) == 0:
            return False
            
        return np.min(scan_array) < self.collision_threshold

    def calculate_movement_consistency(self, current_action, previous_action):
        """Reward consistent movement directions"""
        if previous_action is None:
            return 0.0
            
        action_similarity = np.dot(current_action, previous_action) / (
            np.linalg.norm(current_action) * np.linalg.norm(previous_action) + 1e-6
        )
        
        return self.consistency_reward * max(0, action_similarity)

    def get_reward(self, robot_state, scan, action):
        """Calculate total reward with collision recovery and movement consistency"""
        # Update path history
        current_position = robot_state[:2]
        self.update_path_history(current_position)
        
        # Check for collision
        is_collision = self.check_collision(scan)
        is_near_obstacle, obstacle_distance = self.check_obstacle_proximity(scan)
        
        # Handle collision state
        if is_collision:
            self.collision_count += 1
            self.recovery_mode = True
            self.recovery_steps = 0
            return -self.collision_penalty
        
        # Reset collision count if no collision
        if not is_collision and not is_near_obstacle:
            self.collision_count = 0
        
        total_reward = 0.0
        
        # Reward for exploration
        discretized_pos = self.discretize_position(current_position)
        if discretized_pos not in self.visited_positions:
            self.visited_positions.add(discretized_pos)
            total_reward += self.exploration_reward
        
        # Process LiDAR data
        open_space_reward, open_segments = self.process_lidar(scan)
        total_reward += open_space_reward
        
        # Add movement consistency reward
        consistency_reward = self.calculate_movement_consistency(action, self.previous_action)
        total_reward += consistency_reward
        
        # Penalize being too close to obstacles
        if is_near_obstacle:
            proximity_penalty = self.collision_penalty * (
                1 - obstacle_distance / (self.collision_threshold * 2)
            )
            total_reward -= proximity_penalty
        
        # Update previous action
        self.previous_action = action
        self.previous_scan = scan
        
        return total_reward

    def get_exploration_stats(self):
        """Return statistics about exploration"""
        return {
            'explored_cells': len(self.visited_positions),
            'exploration_coverage': len(self.visited_positions) * (self.grid_resolution ** 2),
            'collision_count': self.collision_count,
            'recovery_mode': self.recovery_mode,
            'recovery_steps': self.recovery_steps
        }