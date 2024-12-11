import numpy as np
import gym
from gym import spaces

class dynamicObstacleEnv(gym.Env):
    def __init__(self, lidar_resolution=360, goal_position = np.array([9, 9])):
        super(dynamicObstacleEnv, self).__init__()

        self.grid_size = 10
        self.num_obstacles = 3
        self.goal_position = goal_position
        self.lidar_resolution = lidar_resolution

    
        # Action Space: [vx, vy, wz]
        self.action_space = spaces.Box(
            low=np.array([-1.0, -1.0, -np.pi]), 
            high=np.array([1.0, 1.0, np.pi]), 
            dtype=np.float64
        )

        # Observation Space: Agent state + obstacles + LiDAR data
        self.observation_space = spaces.Box(
            low=0.0,
            high=self.grid_size,
            shape=(5 + 2 * self.num_obstacles + self.lidar_resolution,),
            dtype=np.float64
        )

        self.reset()

    def reset(self):
        self.agent_position = np.array([0.0, 0.0])
        self.agent_orientation = 0.0
        self.obstacle_positions = np.random.uniform(0, self.grid_size, (self.num_obstacles, 2))
        self.steps = 0
        self.max_steps = 500
        return self._get_observation()
    
    def _get_observation(self):
        lidar_scan = self._simulate_lidar_scan()
        print(self.agent_position.shape)
        print(self.goal_position.shape)
        print(self.obstacle_positions.flatten().shape)
        print(lidar_scan.shape)
        return np.concatenate([
            self.agent_position,
            np.array([self.agent_orientation]),
            self.goal_position,
            self.obstacle_positions.flatten(),
            lidar_scan
        ])

    def step(self, action):
        vx, vy, wz = action
        self.agent_position += np.array([vx, vy]) #Consider 1 sec taken for each step
        self.agent_orientation += wz
        self.agent_orientation %= 2 * np.pi
        self.agent_position = np.clip(self.agent_position, 0, self.grid_size)
        self._move_obstacles()
        reward, done = self._compute_reward_and_done()
        self.steps += 1
        if self.steps >= self.max_steps:
            done = True
        return self._get_observation(), reward, done, {}

    def _move_obstacles(self):
        for i in range(self.num_obstacles):
            self.obstacle_positions[i] += np.random.uniform(-0.1, 0.1, size=2)
            self.obstacle_positions[i] = np.clip(self.obstacle_positions[i], 0, self.grid_size)

    def _compute_reward_and_done(self):
        dist_to_goal = np.linalg.norm(self.agent_position - self.goal_position)
        for obstacle in self.obstacle_positions:
            if np.linalg.norm(self.agent_position - obstacle) < 0.5:
                return -100.0, True
        if dist_to_goal < 0.5:
            return 100.0, True
        return -dist_to_goal * 0.1, False

    

    def _simulate_lidar_scan(self):
        lidar_scan = np.full(self.lidar_resolution, self.grid_size)
        angles = np.linspace(0, 2 * np.pi, self.lidar_resolution, endpoint=False)
        for i, angle in enumerate(angles):
            ray_direction = np.array([np.cos(angle), np.sin(angle)])
            for obstacle in self.obstacle_positions:
                to_obstacle = obstacle - self.agent_position
                proj = np.dot(to_obstacle, ray_direction)
                if proj > 0:
                    perp_dist = np.linalg.norm(to_obstacle - proj * ray_direction)
                    if perp_dist < 0.5:
                        lidar_scan[i] = min(lidar_scan[i], proj)
        return lidar_scan