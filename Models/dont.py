#AGENT.py file
import torch
import torch.nn as nn
import torch.optim as optim
import numpy as np
import random
from collections import deque
import gym
from gym import spaces

# Warehouse Environment
class WarehouseEnv(gym.Env):
    def __init__(self):
        super(WarehouseEnv, self).__init__()
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(2,), dtype=np.float32)
        self.observation_space = spaces.Box(low=0, high=1, shape=(10, 10), dtype=np.float32)
        self.robot_position = [random.randint(0, 9), random.randint(0, 9)]
        # Increased obstacle density
        self.static_objects = np.random.choice([0, 1], size=(10, 10), p=[0.7, 0.3])
        self.goal_position = [9, 9]
        self.step_count = 0
        self.max_steps = 500
        self._visited_cells = np.zeros((10, 10))

    def reset(self):
        self.robot_position = [random.randint(0, 9), random.randint(0, 9)]
        self.step_count = 0
        self._visited_cells = np.zeros((10, 10))
        return self._get_observation()

    def _get_observation(self):
        grid = np.zeros((10, 10))
        grid[self.robot_position[0], self.robot_position[1]] = 1
        grid += self.static_objects
        return grid

    def step(self, action):
        self.step_count += 1
        dx, dy = int(round(action[0])), int(round(action[1]))
        new_x = max(0, min(9, self.robot_position[0] + dx))
        new_y = max(0, min(9, self.robot_position[1] + dy))

        # Check for static obstacle
        if self.static_objects[new_x, new_y] == 1:
            reward = -50  # High penalty for hitting an obstacle
            done = True
            return self._get_observation(), reward, done, {"collision": True, "success": False}

        old_dist = np.linalg.norm(np.array(self.robot_position) - np.array(self.goal_position))
        new_dist = np.linalg.norm(np.array([new_x, new_y]) - np.array(self.goal_position))

        self.robot_position = [new_x, new_y]

        reward = -1
        reward += (old_dist - new_dist) * 10

        if self._visited_cells[new_x, new_y] == 0:
            reward += 5
            self._visited_cells[new_x, new_y] = 1

        if self.robot_position == self.goal_position:
            reward += 100
            done = True
            return self._get_observation(), reward, done, {"collision": False, "success": True}
        elif self.step_count >= self.max_steps:
            done = True
            return self._get_observation(), reward, done, {"collision": False, "success": False}
        else:
            done = False

        return self._get_observation(), reward, done, {"collision": False, "success": False}


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

        self.replay_buffer = deque(maxlen=100000)
        self.batch_size = 256
        self.gamma = 0.99
        self.tau = 0.005
        self.policy_noise = 0.2
        self.noise_clip = 0.5
        self.policy_freq = 2
        self.total_it = 0

    def select_action(self, state, noise_scale=0.1):
        state = torch.FloatTensor(state).unsqueeze(0).to("cpu")
        action = self.actor(state).cpu().data.numpy().flatten()
        action += np.random.normal(0, noise_scale, size=action.shape)
        return np.clip(action, -1, 1)

    def train(self):
        if len(self.replay_buffer) < self.batch_size:
            return

        batch = random.sample(self.replay_buffer, self.batch_size)
        state, action, next_state, reward, not_done = zip(*batch)

        state = torch.FloatTensor(state).to("cpu")
        action = torch.FloatTensor(action).to("cpu")
        next_state = torch.FloatTensor(next_state).to("cpu")
        reward = torch.FloatTensor(reward).unsqueeze(-1).to("cpu")
        not_done = torch.FloatTensor(not_done).unsqueeze(-1).to("cpu")

        with torch.no_grad():
            noise = (torch.randn_like(action) * self.policy_noise).clamp(-self.noise_clip, self.noise_clip)
            next_action = (self.actor_target(next_state) + noise).clamp(-1, 1)

            target_q1, target_q2 = self.critic_target(next_state, next_action)
            target_q = reward + not_done * self.gamma * torch.min(target_q1, target_q2)

        current_q1, current_q2 = self.critic(state, action)
        critic_loss = nn.MSELoss()(current_q1, target_q) + nn.MSELoss()(current_q2, target_q)

        self.critic_optimizer.zero_grad()
        critic_loss.backward()
        self.critic_optimizer.step()

        if self.total_it % self.policy_freq == 0:
            actor_loss = -self.critic.q1_only(state, self.actor(state)).mean()
            self.actor_optimizer.zero_grad()
            actor_loss.backward()
            self.actor_optimizer.step()

            for param, target_param in zip(self.actor.parameters(), self.actor_target.parameters()):
                target_param.data.copy_(self.tau * param.data + (1 - self.tau) * target_param.data)

            for param, target_param in zip(self.critic.parameters(), self.critic_target.parameters()):
                target_param.data.copy_(self.tau * param.data + (1 - self.tau) * target_param.data)

        self.total_it += 1


# Training Loop
env = WarehouseEnv()
state_dim = np.prod(env.observation_space.shape)
action_dim = env.action_space.shape[0]
max_action = float(env.action_space.high[0])

agent = TD3(state_dim, action_dim, max_action)

num_episodes = 10
collision_count_total = 0
success_count = 0

for episode in range(num_episodes):
    state = env.reset().flatten()
    collision_count = 0
    success = False

    for step in range(env.max_steps):
        action = agent.select_action(state, noise_scale=0.2)
        next_state, reward, done, info = env.step(action)
        agent.replay_buffer.append((state, action, next_state.flatten(), reward, float(not done)))

        if info["collision"]:
            collision_count += 1
        if info["success"]:
            success = True

        state = next_state.flatten()

        if len(agent.replay_buffer) > agent.batch_size:
            agent.train()

        if done:
            break

    collision_count_total += collision_count
    if success:
        success_count += 1

    print(f"Episode {episode}: Collisions: {collision_count}, Success: {success}")

collision_rate = collision_count_total / (num_episodes * env.max_steps)
success_rate = success_count / num_episodes

print(f"\nOverall Collision Rate: {collision_rate:.2f}")
print(f"Overall Success Rate: {success_rate:.2f}")
