import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
import matplotlib.pyplot as plt
from gym import spaces
import gym

# ================= Environment: dynamicObstacleEnv =================
class dynamicObstacleEnv(gym.Env):
    def __init__(self, lidar_resolution=360, goal_position=np.array([9, 9])):
        super(dynamicObstacleEnv, self).__init__()
        self.grid_size = 10
        self.num_obstacles = 3
        self.goal_position = goal_position
        self.lidar_resolution = lidar_resolution

        # Precompute angles for lidar
        self.lidar_angles = np.linspace(0, 2 * np.pi, self.lidar_resolution, endpoint=False)

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
        self._prev_dist_to_goal = np.linalg.norm(self.agent_position - self.goal_position)
        return self._get_observation()

    def _get_observation(self):
        lidar_scan = self._simulate_lidar_scan()
        return np.concatenate([
            self.agent_position,
            np.array([self.agent_orientation]),
            self.goal_position,
            self.obstacle_positions.flatten(),
            lidar_scan
        ])

    def step(self, action):
        vx, vy, wz = np.clip(action, self.action_space.low, self.action_space.high)
        self.agent_position += np.array([vx, vy])
        self.agent_orientation += wz
        self.agent_orientation %= 2 * np.pi
        self.agent_position = np.clip(self.agent_position, 0, self.grid_size)
        self._move_obstacles()
        reward, done = self._compute_reward_and_done()
        self.steps += 1
        if self.steps >= self.max_steps:
            return self._get_observation(), -50.0, True, {}
        return self._get_observation(), reward, done, {}

    def _move_obstacles(self):
        for i in range(self.num_obstacles):
            self.obstacle_positions[i] += np.random.uniform(-0.1, 0.1, size=2)
            self.obstacle_positions[i] = np.clip(self.obstacle_positions[i], 0, self.grid_size)

    def _compute_reward_and_done(self):
        dist_to_goal = np.linalg.norm(self.agent_position - self.goal_position)
        reward = (self._prev_dist_to_goal - dist_to_goal) * 10
        self._prev_dist_to_goal = dist_to_goal

        for obstacle in self.obstacle_positions:
            if np.linalg.norm(self.agent_position - obstacle) < 0.5:
                return -100.0, True

        if dist_to_goal < 0.5:
            return 500.0, True

        return reward - 0.1, False

    def _simulate_lidar_scan(self):
        lidar_scan = np.full(self.lidar_resolution, self.grid_size)
        for i, angle in enumerate(self.lidar_angles):
            ray_direction = np.array([np.cos(angle), np.sin(angle)])
            for obstacle in self.obstacle_positions:
                to_obstacle = obstacle - self.agent_position
                proj = np.dot(to_obstacle, ray_direction)
                if 0 < proj < self.grid_size:
                    perp_dist = np.linalg.norm(to_obstacle - proj * ray_direction)
                    if perp_dist < 0.5:
                        lidar_scan[i] = min(lidar_scan[i], proj)
        return lidar_scan


# ================= Model: PPO =================
class PPOModel(nn.Module):
    def __init__(self, state_dim, action_dim, hidden_dim=256):
        super(PPOModel, self).__init__()
        self.fc1 = nn.Linear(state_dim, hidden_dim)
        self.fc2 = nn.Linear(hidden_dim, hidden_dim)
        self.actor_mean = nn.Linear(hidden_dim, action_dim)
        self.actor_std = nn.Linear(hidden_dim, action_dim)
        self.critic = nn.Linear(hidden_dim, 1)

    def forward(self, state):
        x = torch.relu(self.fc1(state))
        x = torch.relu(self.fc2(x))
        return x

    def get_action(self, state):
        x = self.forward(state)
        mean = self.actor_mean(x)
        std = torch.clamp(torch.exp(self.actor_std(x)), 0.1, 2.0)
        action_distribution = torch.distributions.Normal(mean, std)
        action = action_distribution.sample()
        return torch.clamp(action, -1.0, 1.0), action_distribution.log_prob(action).sum(dim=-1), action_distribution.entropy().sum(dim=-1)

    def get_value(self, state):
        x = self.forward(state)
        return self.critic(x)


# ================= Training Loop =================
def train_ppo(env, model, num_episodes=500, gamma=0.99, lr=0.0001):
    optimizer = optim.Adam(model.parameters(), lr=lr)
    episode_rewards = []
    success_rates = []

    for episode in range(num_episodes):
        state = env.reset()
        done = False
        total_reward = 0
        success = 0

        # Lists to store trajectory data
        states_list = []
        actions_list = []
        rewards_list = []
        log_probs_list = []
        dones_list = []

        # Collect trajectory
        while not done:
            state_tensor = torch.FloatTensor(state).unsqueeze(0)

            # Collect action and its log probability
            with torch.no_grad():
                action, log_prob, _ = model.get_action(state_tensor)

            # Step environment
            next_state, reward, done, info = env.step(action.detach().numpy()[0])

            # Check if success (reaching goal)
            if reward == 500.0:
                success = 1

            # Store trajectory data
            states_list.append(state)
            actions_list.append(action.detach().numpy()[0])
            rewards_list.append(reward)
            log_probs_list.append(log_prob)
            dones_list.append(done)

            total_reward += reward
            state = next_state

        # Convert lists to numpy arrays first
        states = np.array(states_list)
        actions = np.array(actions_list)
        rewards = np.array(rewards_list)
        dones = np.array(dones_list)
        old_log_probs = torch.stack(log_probs_list).detach()

        # Convert to tensors
        states_tensor = torch.FloatTensor(states)
        actions_tensor = torch.FloatTensor(actions)
        rewards_tensor = torch.FloatTensor(rewards)

        # Compute returns and advantages
        returns = np.zeros_like(rewards)
        G = 0
        for t in reversed(range(len(rewards))):
            if dones[t]:
                G = 0
            G = rewards[t] + gamma * G
            returns[t] = G

        # Compute values
        with torch.no_grad():
            values = model.get_value(states_tensor).squeeze().numpy()
            advantages = returns - values

        # Normalize advantages
        advantages = (advantages - advantages.mean()) / (advantages.std() + 1e-8)

        # Convert to tensors for loss computation
        returns_tensor = torch.FloatTensor(returns)
        advantages_tensor = torch.FloatTensor(advantages)

        # Compute losses
        for _ in range(3):  # PPO epochs
            # Compute current policy probabilities
            _, new_log_probs, entropy = model.get_action(states_tensor)

            # Compute policy ratio
            ratio = torch.exp(new_log_probs - old_log_probs)

            # Compute policy loss with clipping
            surr1 = ratio * advantages_tensor
            surr2 = torch.clamp(ratio, 1-0.2, 1+0.2) * advantages_tensor
            policy_loss = -torch.min(surr1, surr2).mean()

            # Value loss
            value_loss = (returns_tensor - model.get_value(states_tensor).squeeze()).pow(2).mean()

            # Total loss
            loss = policy_loss + 0.5 * value_loss - 0.01 * entropy.mean()

            # Optimize
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()

        episode_rewards.append(total_reward)
        success_rates.append(success)

        if episode % 10 == 0:
            avg_reward = np.mean(episode_rewards[-10:])
            success_rate = np.mean(success_rates[-10:]) * 100
            print(f"Episode {episode}, Average Reward: {avg_reward:.2f}, Success Rate: {success_rate:.2f}%")

    return model, episode_rewards, success_rates


if __name__ == "__main__":
    lidar_resolution = 360
    env = dynamicObstacleEnv(lidar_resolution=lidar_resolution)
    state_dim = env.observation_space.shape[0]
    action_dim = env.action_space.shape[0]
    model = PPOModel(state_dim, action_dim)
    model, rewards, success_rates = train_ppo(env, model, num_episodes=500)

    # Plot rewards
    plt.figure()
    plt.plot(rewards)
    plt.title("Training Rewards")
    plt.xlabel("Episodes")
    plt.ylabel("Total Reward")
    plt.show()

    # Plot success rates
    plt.figure()
    plt.plot([np.mean(success_rates[max(0, i-10):i+1]) * 100 for i in range(len(success_rates))])
    plt.title("Success Rate")
    plt.xlabel("Episodes")
    plt.ylabel("Success Rate (%)")
    plt.show()
