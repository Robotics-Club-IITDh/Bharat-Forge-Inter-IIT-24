import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
import matplotlib.pyplot as plt
from gym import spaces
import gym

# ================= Environment: StaticObstacleEnv =================
class StaticObstacleEnv(gym.Env):
    def __init__(self, lidar_resolution=360, goal_position=np.array([9, 9])):
        super(StaticObstacleEnv, self).__init__()
        self.grid_size = 10
        self.num_obstacles = 4
        self.goal_position = goal_position
        self.lidar_resolution = lidar_resolution

        # Precompute angles for lidar
        self.lidar_angles = np.linspace(0, 2 * np.pi, self.lidar_resolution, endpoint=False)

        # Action Space: [vx, vy]
        self.action_space = spaces.Box(
            low=np.array([-1.0, -1.0]),
            high=np.array([1.0, 1.0]),
            dtype=np.float32
        )

        # Observation Space: Agent state + obstacles + LiDAR data
        self.observation_space = spaces.Box(
            low=0.0,
            high=self.grid_size,
            shape=(5 + 2 * self.num_obstacles + self.lidar_resolution,),
            dtype=np.float32
        )

        self.reset()

    def reset(self):
        self.agent_position = np.array([1.0, 1.0])  # Start position
        self.agent_orientation = 0.0

        # Place static obstacles
        self.obstacle_positions = np.random.uniform(0, self.grid_size, (self.num_obstacles, 2))
        self.steps = 0
        self.max_steps = 200
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
        vx, vy = np.clip(action, self.action_space.low, self.action_space.high)
        self.agent_position += np.array([vx, vy])
        self.agent_position = np.clip(self.agent_position, 0, self.grid_size)

        dist_to_goal = np.linalg.norm(self.agent_position - self.goal_position)
        min_obstacle_dist = min(np.linalg.norm(self.agent_position - obs) for obs in self.obstacle_positions)

        reward, done = self._compute_reward_and_done(dist_to_goal, min_obstacle_dist)

        self.steps += 1
        if self.steps >= self.max_steps:
            done = True

        return self._get_observation(), reward, done, {'success': dist_to_goal < 0.5}

    def _compute_reward_and_done(self, dist_to_goal, min_obstacle_dist):
        if min_obstacle_dist < 0.5:
            return -50.0, True  # Reduced collision penalty

        if dist_to_goal < 0.5:
            return 150.0, True  # Increased goal-reaching reward

        # Step survival reward
        step_reward = 0.5

        # Reward for reducing the distance to the goal
        proximity_reward = 2 * max(0, 1 - dist_to_goal / self.grid_size)

        # Distance penalty (reduced weight)
        distance_penalty = -0.01 * dist_to_goal

        total_reward = step_reward + proximity_reward + distance_penalty
        return total_reward, False

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
        self.layer_norm_input = nn.LayerNorm(state_dim)
        self.fc1 = nn.Linear(state_dim, hidden_dim)
        self.fc2 = nn.Linear(hidden_dim, hidden_dim)
        self.fc3 = nn.Linear(hidden_dim, hidden_dim)
        self.actor_mean = nn.Linear(hidden_dim, action_dim)
        self.actor_std = nn.Linear(hidden_dim, action_dim)
        self.critic = nn.Linear(hidden_dim, 1)
        self._init_weights()

    def _init_weights(self):
        for m in self.modules():
            if isinstance(m, nn.Linear):
                nn.init.xavier_uniform_(m.weight)
                if m.bias is not None:
                    nn.init.constant_(m.bias, 0)

    def forward(self, state):
        x = self.layer_norm_input(state)
        x = torch.tanh(self.fc1(x))
        x = torch.tanh(self.fc2(x))
        x = torch.tanh(self.fc3(x))
        return x

    def get_action(self, state):
        x = self.forward(state)
        mean = torch.tanh(self.actor_mean(x))  # Ensure mean is bounded
        log_std = torch.clamp(self.actor_std(x), min=-5, max=2)  # Restrict range
        std = torch.exp(log_std)

        # Replace NaN values with defaults
        mean = torch.where(torch.isnan(mean), torch.zeros_like(mean), mean)
        std = torch.where(torch.isnan(std), torch.full_like(std, 1e-6), std)

        # Create distribution
        dist = torch.distributions.Normal(mean, std)
        action = dist.rsample()  # Sample action
        action = torch.clamp(action, -1.0, 1.0)  # Ensure action is valid
        log_prob = dist.log_prob(action).sum(dim=-1)
        entropy = dist.entropy().mean()
        return action, log_prob, entropy

    def get_value(self, state):
        x = self.forward(state)
        return self.critic(x)


# ================= Training Loop =================
def train_ppo(env, model, num_episodes=100, gamma=0.99, lr=0.001, clip_ratio=0.2):  # Reduced to 100 episodes
    optimizer = optim.Adam(model.parameters(), lr=lr, eps=1e-5)
    episode_rewards = []
    success_records = []  # Track successes (1 for success, 0 for failure)

    for episode in range(num_episodes):
        state = env.reset()
        done = False
        total_reward = 0
        log_probs, values, rewards, states, actions = [], [], [], [], []
        success = 0  # Track success for this episode

        while not done:
            state_tensor = torch.FloatTensor(state).unsqueeze(0)
            action, log_prob, _ = model.get_action(state_tensor)
            value = model.get_value(state_tensor)
            next_state, reward, done, info = env.step(action.detach().numpy()[0])
            states.append(state_tensor)
            actions.append(action)
            log_probs.append(log_prob)
            values.append(value)
            rewards.append(reward)
            total_reward += reward
            state = next_state

            # Check success condition
            if done and info.get('success', False):
                success = 1  # Episode successful

        # Append success record for the episode
        success_records.append(success)

        # Compute returns and advantages
        values = torch.cat(values)
        rewards = torch.tensor(rewards, dtype=torch.float32)
        returns, advantages = compute_gae(rewards, values, gamma)

        for _ in range(3):
            new_values = model.get_value(torch.cat(states))
            new_log_probs = torch.cat([model.get_action(s)[1] for s in states])
            ratios = torch.exp(new_log_probs - torch.cat(log_probs).detach())
            surr1 = ratios * advantages
            surr2 = torch.clamp(ratios, 1 - clip_ratio, 1 + clip_ratio) * advantages
            policy_loss = -torch.min(surr1, surr2).mean()
            value_loss = (new_values.squeeze() - returns).pow(2).mean()
            loss = policy_loss + 0.5 * value_loss
            optimizer.zero_grad()
            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), 0.5)
            optimizer.step()

        # Track rewards for this episode
        episode_rewards.append(total_reward)

        # Log every 10 episodes
        if (episode + 1) % 10 == 0:
            avg_reward = np.mean(episode_rewards[-10:])
            success_rate = np.mean(success_records[-10:]) * 100  # Success rate in percentage
            print(f"Episode {episode+1}, Avg Reward: {avg_reward:.2f}, Success Rate: {success_rate:.2f}%")

    # Plot success rate
    plot_success_rate(success_records)

    return model

def plot_success_rate(success_records):
    success_rate = [np.mean(success_records[max(0, i-9):i+1]) * 100 for i in range(len(success_records))]
    plt.figure(figsize=(10, 5))
    plt.plot(success_rate, label="Success Rate (Moving Avg.)", color='b')
    plt.xlabel("Episodes")
    plt.ylabel("Success Rate (%)")
    plt.title("Success Rate vs Episodes")
    plt.legend()
    plt.grid()
    plt.show()

def compute_gae(rewards, values, gamma, lam=0.95):
    returns = []
    advantages = []
    gae = 0
    for i in reversed(range(len(rewards))):
        delta = rewards[i] + gamma * (values[i+1] if i+1 < len(values) else 0) - values[i]
        gae = delta + gamma * lam * gae
        advantages.insert(0, gae)
        returns.insert(0, rewards[i] + gamma * (returns[0] if returns else 0))
    returns = torch.tensor(returns)
    advantages = torch.tensor(advantages)

    # Fix for small or zero std in advantages
    if advantages.std() < 1e-6:
        advantages = advantages - advantages.mean()
    else:
        advantages = (advantages - advantages.mean()) / (advantages.std() + 1e-8)

    return returns, advantages

if __name__ == "__main__":
    lidar_resolution = 360
    env = StaticObstacleEnv(lidar_resolution=lidar_resolution)
    state_dim = env.observation_space.shape[0]
    action_dim = env.action_space.shape[0]
    model = PPOModel(state_dim, action_dim)
    model = train_ppo(env, model)
