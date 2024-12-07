import torch
import torch.nn as nn
import torch.optim as optim
import numpy as np
import gym  # Or integrate with ROS2 environment
from collections import deque

class PPOModel(nn.Module):
    def __init__(self, state_dim, action_dim, hidden_dim=256):
        super(PPOModel, self).__init__()
        self.fc1 = nn.Linear(state_dim, hidden_dim)
        self.fc2 = nn.Linear(hidden_dim, hidden_dim)
        self.actor = nn.Linear(hidden_dim, action_dim)
        self.critic = nn.Linear(hidden_dim, 1)

    def forward(self, state):
        x = torch.relu(self.fc1(state))
        x = torch.relu(self.fc2(x))
        action_probs = torch.softmax(self.actor(x), dim=-1)
        action = torch.distributions.Categorical(action_probs).sample()
        value = self.critic(x)
        return action, value


class PPOTrainer:
    def __init__(self, env, model, gamma=0.99, clip_epsilon=0.2, lr=3e-4, policy_epochs=6, entropy_coeff=0.01):
        self.env = env
        self.model = model
        self.optimizer = optim.Adam(self.model.parameters(), lr=lr)
        self.gamma = gamma
        self.clip_epsilon = clip_epsilon
        self.policy_epochs = policy_epochs
        self.entropy_coeff = entropy_coeff
        self.buffer = []  # Replay buffer

    def collect_trajectory(self, max_steps):
        state = self.env.reset()[0]  # Modify reset() to extract state
        trajectory = []
        for _ in range(max_steps):
            state_tensor = torch.tensor(state, dtype=torch.float32).unsqueeze(0)
            action, value = self.model(state_tensor)
            action = action.item()

            next_state, reward, done, _, info = self.env.step(action)
            trajectory.append((state, action, reward, next_state, done))

            state = next_state
            if done:
                break
        self.buffer.extend(trajectory)

    def compute_advantages(self, rewards, values, dones):
        advantages = []
        returns = []
        G = 0
        for reward, value, done in zip(reversed(rewards), reversed(values), reversed(dones)):
            G = reward + self.gamma * G * (1 - done)
            returns.insert(0, G)
            advantages.insert(0, G - value)
        return torch.tensor(returns, dtype=torch.float32), torch.tensor(advantages, dtype=torch.float32)

    def train(self, batch_size=64):
        # Process trajectory from buffer
        states, actions, rewards, next_states, dones = zip(*self.buffer)
        states = torch.tensor(np.array(states), dtype=torch.float32)
        actions = torch.tensor(np.array(actions), dtype=torch.float32)
        rewards = torch.tensor(np.array(rewards), dtype=torch.float32)
        dones = torch.tensor(np.array(dones), dtype=torch.float32)

        _, values = self.model(states)
        values = values.squeeze()

        # Compute advantages
        returns, advantages = self.compute_advantages(rewards, values, dones)

        for _ in range(self.policy_epochs):
            # PPO Optimization Loop
            action_preds, value_preds = self.model(states)
            value_preds = value_preds.squeeze()

            # Policy Loss
            ratios = (action_preds - actions).pow(2).mean(dim=-1)  # Approximate policy divergence
            clipped_ratios = torch.clamp(ratios, 1 - self.clip_epsilon, 1 + self.clip_epsilon)
            actor_loss = -torch.min(ratios * advantages, clipped_ratios * advantages).mean()

            # Value Loss
            critic_loss = nn.MSELoss()(value_preds, returns)

            # Entropy Loss
            entropy_loss = -self.entropy_coeff * torch.sum(-action_preds * torch.log(action_preds + 1e-8))

            loss = actor_loss + 0.5 * critic_loss + entropy_loss

            self.optimizer.zero_grad()
            loss.backward()
            self.optimizer.step()

        self.buffer = []  # Clear buffer after training

    def save_weights(self, filename="ppo_weights.pth"):
        """Save only the model's weights (state_dict)."""
        torch.save(self.model.state_dict(), filename)
        print(f"Weights saved to {filename}")


# Main Training Loop
if __name__ == "__main__":
    env = gym.make('CartPole-v1')  # Replace with ROS2 environment or custom simulation
    state_dim = env.observation_space.shape[0]
    action_dim = env.action_space.n if hasattr(env.action_space, 'n') else env.action_space.shape[0]

    model = PPOModel(state_dim, action_dim)
    trainer = PPOTrainer(env, model)

    episodes = 1000
    max_steps = 200
    for episode in range(episodes):
        trainer.collect_trajectory(max_steps)
        trainer.train()

        # Save weights every 50 episodes
        if episode % 50 == 0:
            trainer.save_weights("ppo_weights.pth")
            print(f"Episode {episode}: Weights saved")
