import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import medfilt

class RewardLiDAR:
    def __init__(self, num_beams, max_range, collision_threshold, collision_penalty=100.0,
                 action_cost=0.1, raycast_cost=20.0, fov=np.pi, angular_resolution=None):
        self.num_beams = num_beams
        self.max_range = max_range
        self.collision_threshold = collision_threshold
        self.collision_penalty = collision_penalty
        self.action_cost = action_cost
        self.raycast_cost = raycast_cost
        self.fov = fov  # Field of view in radians
        self.angular_resolution = angular_resolution or (fov / num_beams)
        self.initializeRaycastRewardWeights()
        self.large_constant = 1e5
        self.tol = 1e-3
        self.cutoff = max_range  # Adjust cutoff dynamically to max_range

    def initializeRaycastRewardWeights(self):
        # Compute beam angles based on FOV and number of beams or angular resolution
        if self.angular_resolution:
            self.num_beams = int(self.fov / self.angular_resolution)
        angles = np.linspace(-self.fov / 2, self.fov / 2, self.num_beams)
        self.raycast_reward_weights = np.cos(angles) + 0.2
        self.raycast_reward_weights = -self.raycast_cost * self.raycast_reward_weights / self.raycast_reward_weights.sum()

    def denoiseLiDAR(self, lidar_ranges):
        # Apply median filter to denoise LiDAR data
        return medfilt(lidar_ranges, kernel_size=3)

    def checkInCollision(self, lidar_ranges):
        # Collision if any LiDAR range is below the threshold
        if np.min(lidar_ranges) < self.collision_threshold:
            return True
        return False

    def computeReward(self, S, u):
        car_state, lidar_ranges = S
        lidar_ranges = self.denoiseLiDAR(lidar_ranges)

        if self.checkInCollision(lidar_ranges):
            return -self.collision_penalty

        reward = -self.action_cost * np.linalg.norm(u)
        reward += self.computeRaycastReward(S, u)
        return reward

    def computeRaycastReward(self, S, u):
        _, lidar_ranges = S
        lidar_ranges = self.denoiseLiDAR(lidar_ranges)

        # Replace inf values with large constant
        lidar_ranges = np.where(np.isinf(lidar_ranges), self.large_constant, lidar_ranges)

        # Truncate and normalize ranges
        inverse_truncated = self.inverseTruncate(lidar_ranges)
        return np.dot(self.raycast_reward_weights, inverse_truncated)

    def inverseTruncate(self, ranges):
        # Truncate ranges above cutoff or below collision_threshold
        ranges_clipped = np.clip(ranges, self.collision_threshold, self.cutoff)
        return 1.0 / ranges_clipped  # Inverse weighting for closer objects

    def computeRewardFromLiDAR(self, lidar_ranges, u=0.0):
        car_state = 0.0  # Placeholder
        S = (car_state, lidar_ranges)
        return self.computeReward(S, u)

    def plotRaycastRewardWeights(self):
        angles = np.linspace(-self.fov / 2, self.fov / 2, self.num_beams)
        plt.plot(angles, self.raycast_reward_weights)
        plt.xlabel("Beam Angle (radians)")
        plt.ylabel("LiDAR Reward Weight")
        plt.title("Raycast Reward Weights Distribution")
        plt.show()

