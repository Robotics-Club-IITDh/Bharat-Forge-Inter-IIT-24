import torch
import torch.nn as nn
import torch.optim as optim
import numpy as np
import random
from collections import deque
import gym
from gym import spaces

# ReplayBuffer Class
class ReplayBuffer:
    def __init__(self, max_size, state_dim, action_dim):
        self.max_size = max_size
        self.ptr = 0
        self.size = 0
        self.state = np.zeros((max_size, state_dim))
        self.action = np.zeros((max_size, action_dim))
        self.reward = np.zeros((max_size, 1))
        self.next_state = np.zeros((max_size, state_dim))
        self.not_done = np.zeros((max_size, 1))

    def store(self, state, action, reward, next_state, done):
        self.state[self.ptr] = state
        self.action[self.ptr] = action
        self.reward[self.ptr] = reward
        self.next_state[self.ptr] = next_state
        self.not_done[self.ptr] = 1.0 - done

        self.ptr = (self.ptr + 1) % self.max_size
        self.size = min(self.size + 1, self.max_size)

    def sample(self, batch_size):
        indices = np.random.randint(0, self.size, size=batch_size)
        return (
            self.state[indices],
            self.action[indices],
            self.reward[indices],
            self.next_state[indices],
            self.not_done[indices],
        )

# Actor Network
class Actor(nn.Module):
    def __init__(self, state_dim, action_dim, max_action):
        super(Actor, self).__init__()
        self.network = nn.Sequential(
            nn.Linear(state_dim, 256),
            nn.ReLU(),
            nn.Linear(256, 256),
            nn.ReLU(),
            nn.Linear(256, action_dim),
            nn.Tanh()
        )
        self.max_action = max_action

    def forward(self, x):
        return self.max_action * self.network(x)

# Critic Network
class Critic(nn.Module):
    def __init__(self, state_dim, action_dim):
        super(Critic, self).__init__()
        self.q1 = nn.Sequential(
            nn.Linear(state_dim + action_dim, 256),
            nn.ReLU(),
            nn.Linear(256, 256),
            nn.ReLU(),
            nn.Linear(256, 1)
        )
        self.q2 = nn.Sequential(
            nn.Linear(state_dim + action_dim, 256),
            nn.ReLU(),
            nn.Linear(256, 256),
            nn.ReLU(),
            nn.Linear(256, 1)
        )

    def forward(self, state, action):
        sa = torch.cat([state, action], dim=-1)
        q1 = self.q1(sa)
        q2 = self.q2(sa)
        return q1, q2

    def q1_only(self, state, action):
        sa = torch.cat([state, action], dim=-1)
        return self.q1(sa)

# TD3 Algorithm
class TD3:
    def __init__(self, state_dim, action_dim, max_action):
        self.actor = Actor(state_dim, action_dim, max_action).to("cpu")
        self.actor_target = Actor(state_dim, action_dim, max_action).to("cpu")
        self.actor_target.load_state_dict(self.actor.state_dict())
        self.actor_optimizer = optim.Adam(self.actor.parameters(), lr=3e-4)

        self.critic = Critic(state_dim, action_dim).to("cpu")
        self.critic_target = Critic(state_dim, action_dim).to("cpu")
        self.critic_target.load_state_dict(self.critic.state_dict())
        self.critic_optimizer = optim.Adam(self.critic.parameters(), lr=3e-4)

        self.max_action = max_action
        self.policy_noise = 0.2
        self.noise_clip = 0.5
        self.tau = 0.005
        self.gamma = 0.99
        self.policy_freq = 2
        self.total_it = 0

    def select_action(self, state):
        state = torch.FloatTensor(state).unsqueeze(0).to("cpu")
        return self.actor(state).cpu().data.numpy().flatten()

    def train(self, replay_buffer, batch_size):
        if replay_buffer.size < batch_size:
            return

        state, action, reward, next_state, not_done = replay_buffer.sample(batch_size)
        state = torch.FloatTensor(state).to("cpu")
        action = torch.FloatTensor(action).to("cpu")
        reward = torch.FloatTensor(reward).to("cpu")
        next_state = torch.FloatTensor(next_state).to("cpu")
        not_done = torch.FloatTensor(not_done).to("cpu")

        # Update Critic
        with torch.no_grad():
            noise = (
                torch.randn_like(action) * self.policy_noise
            ).clamp(-self.noise_clip, self.noise_clip)
            next_action = (self.actor_target(next_state) + noise).clamp(-self.max_action, self.max_action)

            target_q1, target_q2 = self.critic_target(next_state, next_action)
            target_q = reward + not_done * self.gamma * torch.min(target_q1, target_q2)

        current_q1, current_q2 = self.critic(state, action)
        critic_loss = nn.MSELoss()(current_q1, target_q) + nn.MSELoss()(current_q2, target_q)

        self.critic_optimizer.zero_grad()
        critic_loss.backward()
        self.critic_optimizer.step()

        # Update Actor
        if self.total_it % self.policy_freq == 0:
            actor_loss = -self.critic.q1_only(state, self.actor(state)).mean()
            self.actor_optimizer.zero_grad()
            actor_loss.backward()
            self.actor_optimizer.step()

            # Update target networks
            for param, target_param in zip(self.actor.parameters(), self.actor_target.parameters()):
                target_param.data.copy_(
                    self.tau * param.data + (1 - self.tau) * target_param.data
                )
            for param, target_param in zip(self.critic.parameters(), self.critic_target.parameters()):
                target_param.data.copy_(
                    self.tau * param.data + (1 - self.tau) * target_param.data
                )

        self.total_it += 1

# Warehouse Environment
class WarehouseEnv(gym.Env):
    def __init__(self):
        super(WarehouseEnv, self).__init__()
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(2,), dtype=np.float32)
        self.observation_space = spaces.Box(low=0, high=1, shape=(6,), dtype=np.float32)
        self.robot_position = [random.randint(0, 9), random.randint(0, 9)]
        self.goal_position = [9, 9]
        self.step_count = 0
        self.max_steps = 500

    def reset(self):
        self.robot_position = [random.randint(0, 9), random.randint(0, 9)]
        self.step_count = 0
        return self._get_observation()

    def _get_observation(self):
        position = np.array(self.robot_position) / 10.0  # Normalize position
        orientation = np.array([1, 0, 0, 0])  # Placeholder for orientation
        return np.concatenate([position, orientation])

    def step(self, action):
        self.step_count += 1
        dx, dy = int(round(action[0])), int(round(action[1]))
        new_x = max(0, min(9, self.robot_position[0] + dx))
        new_y = max(0, min(9, self.robot_position[1] + dy))

        previous_distance = np.linalg.norm(np.array(self.robot_position) - np.array(self.goal_position))
        self.robot_position = [new_x, new_y]
        current_distance = np.linalg.norm(np.array(self.robot_position) - np.array(self.goal_position))

        reward = -0.1  # Step penalty
        reward += 10 * (previous_distance - current_distance)  # Reward progress
        reward -= 0.5 * current_distance  # Distance penalty

        if self.robot_position == self.goal_position:
            return self._get_observation(), 500, True, {"collision": False, "success": True}

        if self.step_count >= self.max_steps:
            return self._get_observation(), reward, True, {"collision": False, "success": False}

        return self._get_observation(), reward, False, {}

# Training Loop
env = WarehouseEnv()
state_dim = env.observation_space.shape[0]
action_dim = env.action_space.shape[0]
max_action = float(env.action_space.high[0])
agent = TD3(state_dim, action_dim, max_action)
replay_buffer = ReplayBuffer(100000, state_dim, action_dim)

num_episodes = 100
batch_size = 256

for episode in range(num_episodes):
    state = env.reset()
    episode_reward = 0

    for step in range(env.max_steps):
        action = agent.select_action(state)
        next_state, reward, done, _ = env.step(action)
        replay_buffer.store(state, action, reward, next_state, done)

        state = next_state
        episode_reward += reward

        if replay_buffer.size > batch_size:
            agent.train(replay_buffer, batch_size)

        if done:
            break

    print(f"Episode {episode + 1}: Total Reward: {episode_reward}")

# Save the trained actor
torch.save(agent.actor.state_dict(), "actor_model.pt")
