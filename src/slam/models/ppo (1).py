import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
import matplotlib.pyplot as plt
from gym import spaces
import gym

# ================= Environment: StaticObstacleEnv =================
class StaticObstacleEnv(gym.Env):
    def __init__(self, lidar_resolution=360, goal_position=np.array([7, 7])):
        super(StaticObstacleEnv, self).__init__()
        self.grid_size = 8  # Reduced grid size
        self.num_obstacles = 3  # Reduced number of obstacles
        self.goal_position = goal_position
        self.lidar_resolution = lidar_resolution
        self.previous_dist = None

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
        self.previous_dist = None  # Reset previous distance

        # Place static obstacles
        self.obstacle_positions = np.random.uniform(1, self.grid_size-1, (self.num_obstacles, 2))
        
        # Ensure obstacles are not too close to start or goal
        for i in range(self.num_obstacles):
            while (np.linalg.norm(self.obstacle_positions[i] - self.agent_position) < 2.0 or 
                   np.linalg.norm(self.obstacle_positions[i] - self.goal_position) < 2.0):
                self.obstacle_positions[i] = np.random.uniform(1, self.grid_size-1, 2)
        
        self.steps = 0
        self.max_steps = 150  # Reduced max steps due to smaller environment
        
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
            reward -= 50  # Penalty for timeout

        return self._get_observation(), reward, done, {'success': dist_to_goal < 0.5}

    def _compute_reward_and_done(self, dist_to_goal, min_obstacle_dist):
        # Stronger penalty for collision
        if min_obstacle_dist < 0.5:
            return -100.0, True
            
        # Higher reward for reaching goal
        if dist_to_goal < 0.5:
            return 200.0, True
            
        # Improved reward shaping
        # Base survival penalty (encourages faster completion)
        step_penalty = -0.1
        
        # Stronger proximity reward
        proximity_reward = 3.0 * (1.0 - dist_to_goal / np.sqrt(2 * self.grid_size**2))
        
        # Obstacle avoidance reward
        obstacle_reward = min(2.0, min_obstacle_dist) if min_obstacle_dist < 2.0 else 0.0
        
        # Progress reward (reward for moving closer to goal)
        if self.previous_dist is None:
            self.previous_dist = dist_to_goal
        progress_reward = self.previous_dist - dist_to_goal
        self.previous_dist = dist_to_goal
        
        total_reward = step_penalty + proximity_reward + obstacle_reward + 2.0 * progress_reward
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
    def __init__(self, state_dim, action_dim, hidden_dim=512):
        super(PPOModel, self).__init__()
        self.dropout = nn.Dropout(0.1)
        
        # Input normalization
        self.layer_norm_input = nn.LayerNorm(state_dim)
        
        # Main network
        self.fc1 = nn.Linear(state_dim, hidden_dim)
        self.layer_norm1 = nn.LayerNorm(hidden_dim)
        
        self.fc2 = nn.Linear(hidden_dim, hidden_dim)
        self.layer_norm2 = nn.LayerNorm(hidden_dim)
        
        self.fc3 = nn.Linear(hidden_dim, hidden_dim//2)
        self.layer_norm3 = nn.LayerNorm(hidden_dim//2)
        
        # Separate action and value streams
        self.actor_fc = nn.Linear(hidden_dim//2, hidden_dim//4)
        self.critic_fc = nn.Linear(hidden_dim//2, hidden_dim//4)
        
        self.actor_mean = nn.Linear(hidden_dim//4, action_dim)
        self.actor_std = nn.Linear(hidden_dim//4, action_dim)
        self.critic = nn.Linear(hidden_dim//4, 1)
        
        self._init_weights()

    def _init_weights(self):
        for m in self.modules():
            if isinstance(m, nn.Linear):
                nn.init.orthogonal_(m.weight, gain=np.sqrt(2))
                if m.bias is not None:
                    nn.init.constant_(m.bias, 0)

    def forward(self, state):
        x = self.layer_norm_input(state)
        
        x = self.fc1(x)
        x = self.layer_norm1(x)
        x = torch.tanh(x)
        x = self.dropout(x)
        
        x = self.fc2(x)
        x = self.layer_norm2(x)
        x = torch.tanh(x)
        x = self.dropout(x)
        
        x = self.fc3(x)
        x = self.layer_norm3(x)
        x = torch.tanh(x)
        
        return x

    def get_action(self, state):
        x = self.forward(state)
        
        # Actor stream
        actor_features = torch.tanh(self.actor_fc(x))
        mean = torch.tanh(self.actor_mean(actor_features))
        log_std = torch.clamp(self.actor_std(actor_features), min=-5, max=2)
        std = torch.exp(log_std)
        
        # Handle numerical instabilities
        mean = torch.where(torch.isnan(mean), torch.zeros_like(mean), mean)
        std = torch.where(torch.isnan(std), torch.full_like(std, 1e-6), std)
        std = torch.where(std < 1e-6, torch.full_like(std, 1e-6), std)

        # Create distribution
        dist = torch.distributions.Normal(mean, std)
        action = dist.rsample()
        action = torch.clamp(action, -1.0, 1.0)
        log_prob = dist.log_prob(action).sum(dim=-1)
        entropy = dist.entropy().mean()
        
        return action, log_prob, entropy

    def get_value(self, state):
        x = self.forward(state)
        critic_features = torch.tanh(self.critic_fc(x))
        return self.critic(critic_features)

def compute_gae(rewards, values, gamma=0.99, lam=0.95):
    returns = []
    advantages = []
    gae = 0
    
    for i in reversed(range(len(rewards))):
        if i == len(rewards) - 1:
            next_value = 0
        else:
            next_value = values[i + 1].item()
            
        delta = rewards[i] + gamma * next_value - values[i].item()
        gae = delta + gamma * lam * gae
        
        advantages.insert(0, gae)
        returns.insert(0, gae + values[i].item())
        
    returns = torch.tensor(returns, dtype=torch.float32)
    advantages = torch.tensor(advantages, dtype=torch.float32)
    
    # Normalize advantages
    if len(advantages) > 1:
        advantages = (advantages - advantages.mean()) / (advantages.std() + 1e-8)
    
    return returns, advantages

def train_ppo(env, model, num_episodes=1000, gamma=0.99, lr=3e-4, clip_ratio=0.2):
    optimizer = optim.Adam(model.parameters(), lr=lr, eps=1e-5, betas=(0.9, 0.999))
    scheduler = optim.lr_scheduler.StepLR(optimizer, step_size=100, gamma=0.9)
    
    episode_rewards = []
    success_records = []
    best_success_rate = 0
    best_model_state = None
    
    for episode in range(num_episodes):
        state = env.reset()
        done = False
        total_reward = 0
        trajectory = {'states': [], 'actions': [], 'rewards': [], 'log_probs': [], 'values': []}
        
        # Collect trajectory
        while not done:
            state_tensor = torch.FloatTensor(state).unsqueeze(0)
            
            with torch.no_grad():
                action, log_prob, _ = model.get_action(state_tensor)
                value = model.get_value(state_tensor)
            
            next_state, reward, done, info = env.step(action.detach().numpy()[0])
            
            trajectory['states'].append(state_tensor)
            trajectory['actions'].append(action)
            trajectory['rewards'].append(reward)
            trajectory['log_probs'].append(log_prob)
            trajectory['values'].append(value)
            
            total_reward += reward
            state = next_state
            
        success_records.append(1 if info.get('success', False) else 0)
        episode_rewards.append(total_reward)
        
        # Convert trajectory to tensors
        states = torch.cat(trajectory['states'])
        actions = torch.cat(trajectory['actions'])
        old_log_probs = torch.stack(trajectory['log_probs'])
        old_values = torch.cat(trajectory['values'])
        rewards = torch.tensor(trajectory['rewards'], dtype=torch.float32)
        
        # Compute returns and advantages
        returns, advantages = compute_gae(trajectory['rewards'], trajectory['values'], gamma)
        
        # PPO update
        for _ in range(5):  # 5 PPO epochs
            # Get current policy outputs
            _, new_log_probs, entropy = model.get_action(states)
            new_values = model.get_value(states)
            
            # Compute policy loss
            ratio = torch.exp(new_log_probs - old_log_probs)
            surr1 = ratio * advantages
            surr2 = torch.clamp(ratio, 1.0 - clip_ratio, 1.0 + clip_ratio) * advantages
            policy_loss = -torch.min(surr1, surr2).mean()
            
            # Compute value loss
            value_loss = 0.5 * (returns - new_values.squeeze()).pow(2).mean()
            
            # Compute entropy loss
            entropy_loss = -0.01 * entropy
            
            # Total loss
            loss = policy_loss + value_loss + entropy_loss
            
            # Update model
            optimizer.zero_grad()
            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), max_norm=0.5)
            optimizer.step()
        
        scheduler.step()
        
        # Logging and saving best model
        if (episode + 1) % 10 == 0:
            recent_success_rate = np.mean(success_records[-10:]) * 100
            avg_reward = np.mean(episode_rewards[-10:])
            print(f"Episode {episode+1}, Success Rate: {recent_success_rate:.1f}%, Avg Reward: {avg_reward:.1f}")
            
            if recent_success_rate > best_success_rate:
                best_success_rate = recent_success_rate
                best_model_state = model.state_dict().copy()
    
    # Restore best model
    if best_model_state is not None:
        model.load_state_dict(best_model_state)
    
    # Plot final results
    plot_training_results(episode_rewards, success_records)
    
    return model

def plot_training_results(episode_rewards, success_records):
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 10))
    
    # Plot rewards
    ax1.plot(episode_rewards, label='Episode Reward', alpha=0.6)
    ax1.plot(np.convolve(episode_rewards, np.ones(10)/10, mode='valid'), 
             label='Moving Average (10 episodes)', linewidth=2)
    ax1.set_xlabel('Episode')
    ax1.set_ylabel('Reward')
    ax1.set_title('Training Rewards')
    ax1.legend()
    ax1.grid(True)
    
    # Plot success rate
    window = 10
    success_rate = [np.mean(success_records[max(0, i-window+1):i+1]) * 100 
                   for i in range(len(success_records))]
    ax2.plot(success_rate, label=f'Success Rate (Moving Avg {window} episodes)', 
             color='green', linewidth=2)
    ax2.set_xlabel('Episode')
    ax2.set_ylabel('Success Rate (%)')
    ax2.set_title('Training Success Rate')
    ax2.legend()
    ax2.grid(True)
    
    plt.tight_layout()
    plt.show()

# Function to evaluate the trained model
def evaluate_model(env, model, num_episodes=50):
    success_count = 0
    total_rewards = []
    
    for episode in range(num_episodes):
        state = env.reset()
        done = False
        episode_reward = 0
        
        while not done:
            with torch.no_grad():
                state_tensor = torch.FloatTensor(state).unsqueeze(0)
                action, _, _ = model.get_action(state_tensor)
            
            next_state, reward, done, info = env.step(action.numpy()[0])
            episode_reward += reward
            state = next_state
            
            if info.get('success', False):
                success_count += 1
                break
        
        total_rewards.append(episode_reward)
    
    success_rate = (success_count / num_episodes) * 100
    avg_reward = np.mean(total_rewards)
    
    print(f"\nEvaluation Results:")
    print(f"Success Rate: {success_rate:.1f}%")
    print(f"Average Reward: {avg_reward:.1f}")
    
    return success_rate, avg_reward

if __name__ == "__main__":
    # Set random seeds for reproducibility
    torch.manual_seed(42)
    np.random.seed(42)
    
    # Create environment and model
    lidar_resolution = 360
    env = StaticObstacleEnv(lidar_resolution=lidar_resolution)
    
    # Get dimensions
    state_dim = env.observation_space.shape[0]
    action_dim = env.action_space.shape[0]
    
    # Create model with improved architecture
    model = PPOModel(state_dim=state_dim, 
                    action_dim=action_dim,
                    hidden_dim=512)  # Increased hidden dim
    
    # Train the model
    print("Starting training...")
    trained_model = train_ppo(env, model,
                            num_episodes=1000,  # Increased episodes
                            gamma=0.99,
                            lr=3e-4,
                            clip_ratio=0.2)
    
    # Evaluate the trained model
    print("\nEvaluating trained model...")
    evaluate_model(env, trained_model)
