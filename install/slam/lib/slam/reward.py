import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import medfilt

class RewardLiDAR:
    def __init__(self, num_beams, max_range, collision_threshold, goal_position,
                 collision_penalty=100.0, action_cost=0.1, raycast_cost=20.0, 
                 goal_reward_scale=50.0, fov=np.pi, angular_resolution=None):
        """
        Initialize RewardLiDAR with parameters.
        Args:
            num_beams (int): Number of LiDAR beams.
            max_range (float): Maximum range of LiDAR.
            collision_threshold (float): Threshold distance for collision.
            goal_position (tuple): Target position (x, y).
            collision_penalty (float): Penalty for collision.
            action_cost (float): Cost for taking an action.
            raycast_cost (float): Weight for raycast-based rewards.
            goal_reward_scale (float): Scaling factor for goal-based reward.
            fov (float): Field of view in radians.
            angular_resolution (float): Angular resolution of LiDAR beams.
        """
        self.num_beams = num_beams
        self.max_range = max_range
        self.collision_threshold = collision_threshold
        self.goal_position = np.array(goal_position)
        self.collision_penalty = collision_penalty
        self.action_cost = action_cost
        self.raycast_cost = raycast_cost
        self.goal_reward_scale = goal_reward_scale
        self.fov = fov  # Field of view in radians
        self.angular_resolution = angular_resolution or (fov / num_beams)
        self.initializeRaycastRewardWeights()
        self.large_constant = 1e5
        self.tol = 1e-3
        self.cutoff = max_range  # Adjust cutoff dynamically to max_range

    def initializeRaycastRewardWeights(self):
        """Initialize weights for raycast-based rewards."""
        angles = np.linspace(-self.fov / 2, self.fov / 2, self.num_beams)
        self.raycast_reward_weights = np.cos(angles) + 0.2
        self.raycast_reward_weights = -self.raycast_cost * self.raycast_reward_weights / self.raycast_reward_weights.sum()

    def denoiseLiDAR(self, lidar_ranges):
        """Apply median filter to denoise LiDAR data."""
        return medfilt(lidar_ranges, kernel_size=3)

    def checkInCollision(self, lidar_ranges):
        """Collision detection based on LiDAR readings."""
        return np.min(lidar_ranges) < self.collision_threshold

    def computeReward(self, S, u):
        """
        Compute the reward for the given state and action.
        Args:
            S (tuple): (robot_state, lidar_ranges)
            u (array): Action (e.g., velocity commands).
        Returns:
            float: Computed reward.
        """
        robot_state, lidar_ranges = S
        lidar_ranges = self.denoiseLiDAR(lidar_ranges)

        # Collision penalty
        if self.checkInCollision(lidar_ranges):
            return -self.collision_penalty

        # Action cost
        reward = -self.action_cost * np.linalg.norm(u)

        # Raycast reward
        reward += self.computeRaycastReward(lidar_ranges)

        # Goal-based reward
        distance_to_goal = np.linalg.norm(self.goal_position - np.array(robot_state[:2]))
        reward += self.goal_reward_scale / (distance_to_goal + 1e-3)  # Avoid division by zero

        return reward

    def computeRaycastReward(self, lidar_ranges):
        """
        Compute reward based on LiDAR raycast data.
        Args:
            lidar_ranges (array): LiDAR distance readings.
        Returns:
            float: Raycast reward.
        """
        lidar_ranges = self.denoiseLiDAR(lidar_ranges)
        lidar_ranges = np.where(np.isinf(lidar_ranges), self.large_constant, lidar_ranges)
        inverse_truncated = self.inverseTruncate(lidar_ranges)
        return np.dot(self.raycast_reward_weights, inverse_truncated)

    def inverseTruncate(self, ranges):
        """Truncate and invert LiDAR ranges for weighting."""
        ranges_clipped = np.clip(ranges, self.collision_threshold, self.cutoff)
        return 1.0 / ranges_clipped  # Inverse weighting for closer objects

    def computeRewardFromLiDAR(self, robot_state, lidar_ranges, u):
        """
        Compute reward directly from LiDAR data and robot state.
        Args:
            robot_state (array): [x, y, orientation].
            lidar_ranges (array): LiDAR distance readings.
            u (array): Action (e.g., velocity commands).
        Returns:
            float: Reward.
        """
        S = (robot_state, lidar_ranges)
        return self.computeReward(S, u)

    def plotRaycastRewardWeights(self):
        """Visualize the raycast reward weights distribution."""
        angles = np.linspace(-self.fov / 2, self.fov / 2, self.num_beams)
        plt.plot(angles, self.raycast_reward_weights)
        plt.xlabel("Beam Angle (radians)")
        plt.ylabel("LiDAR Reward Weight")
        plt.title("Raycast Reward Weights Distribution")
        plt.grid(True)
        plt.show()
