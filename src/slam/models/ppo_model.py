import torch
import torch.nn as nn
import numpy

class PPOModel(nn.Module):
    def __init__(self, state_dim, action_dim, hidden_dim=256):
        super(PPOModel, self).__init__()
        
        # Keep original architecture
        self.fc1 = nn.Linear(state_dim, hidden_dim)
        self.fc2 = nn.Linear(hidden_dim, hidden_dim)
        self.actor = nn.Linear(hidden_dim, action_dim)
        self.critic = nn.Linear(hidden_dim, 1)

    def forward(self, state):
        x = torch.relu(self.fc1(state))
        x = torch.relu(self.fc2(x))
        return x

    def get_action(self, state, lidar_ranges=None):
        """
        Get action with obstacle awareness
        Args:
            state: Input state tensor
            lidar_ranges: Optional LiDAR readings for obstacle detection
        Returns:
            tuple: (linear_vel, angular_vel)
        """
        x = self.forward(state)
        action_logits = self.actor(x)
        probs = torch.softmax(action_logits, dim=-1)
        
        # Base velocities
        linear_scale = 0.2
        angular_scale = 0.3
        
        # Get most likely action
        action_idx = torch.argmax(probs, dim=-1).item()
        
        # Default velocities based on action
        if action_idx == 0:  # Forward
            linear_vel = linear_scale
            angular_vel = 0.0
        else:  # Turn
            linear_vel = linear_scale * 0.5
            angular_vel = angular_scale
            
        return (linear_vel, angular_vel)

    def get_value(self, state):
        x = self.forward(state)
        value = self.critic(x)
        return value

