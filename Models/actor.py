#Actor Network
import torch
import torch.nn as nn
import torch.nn.functional as F
import numpy as np
import os
from collections import deque





class Actor(nn.Module):
    def __init__(self, lidar_dim, state_dim, action_dim, max_action):
        super(Actor, self).__init__()
        self.lidar_encoder = nn.Sequential(
            nn.Linear(lidar_dim, 128),  # Encode LiDAR readings
            nn.ReLU(),
            nn.Linear(128, 128),
            nn.ReLU()
        )
        self.state_encoder = nn.Sequential(
            nn.Linear(state_dim, 128),  # Encode additional state inputs (e.g., velocity, heading)
            nn.ReLU(),
            nn.Linear(128, 128),
            nn.ReLU()
        )
        self.fusion = nn.Sequential(
            nn.Linear(128 + 128, 256),  # Combine encoded LiDAR and state features
            nn.ReLU(),
            nn.Linear(256, 256),
            nn.ReLU(),
            nn.Linear(256, action_dim),
            nn.Tanh()
        )
        self.max_action = max_action

    def forward(self, lidar, state):
        lidar_features = self.lidar_encoder(lidar)
        state_features = self.state_encoder(state)
        combined = torch.cat([lidar_features, state_features], dim=-1)
        return self.max_action * self.fusion(combined)


# Critic Network
class Critic(nn.Module):
    def __init__(self, lidar_dim, state_dim, action_dim):
        super(Critic, self).__init__()
        self.lidar_encoder = nn.Sequential(
            nn.Linear(lidar_dim, 128),  # Encode LiDAR readings
            nn.ReLU(),
            nn.Linear(128, 128),
            nn.ReLU()
        )
        self.state_encoder = nn.Sequential(
            nn.Linear(state_dim, 128),  # Encode additional state inputs (e.g., velocity, heading)
            nn.ReLU(),
            nn.Linear(128, 128),
            nn.ReLU()
        )
        self.q1 = nn.Sequential(
            nn.Linear(128 + 128 + action_dim, 256),  # Combine encoded features and action
            nn.ReLU(),
            nn.Linear(256, 256),
            nn.ReLU(),
            nn.Linear(256, 1)
        )
        self.q2 = nn.Sequential(
            nn.Linear(128 + 128 + action_dim, 256),
            nn.ReLU(),
            nn.Linear(256, 256),
            nn.ReLU(),
            nn.Linear(256, 1)
        )

    def forward(self, lidar, state, action):
        lidar_features = self.lidar_encoder(lidar)
        state_features = self.state_encoder(state)
        combined = torch.cat([lidar_features, state_features, action], dim=-1)
        q1 = self.q1(combined)
        q2 = self.q2(combined)
        return q1, q2

    def q1_only(self, lidar, state, action):
        lidar_features = self.lidar_encoder(lidar)
        state_features = self.state_encoder(state)
        combined = torch.cat([lidar_features, state_features, action], dim=-1)
        return self.q1(combined)
    def forward(self, state, action):
        sa = torch.cat([state, action], dim=-1)
        q1 = self.q1(sa)
        q2 = self.q2(sa)
        return q1, q2

    def q1_only(self, state, action):
        sa = torch.cat([state, action], dim=-1)
        return self.q1(sa)
