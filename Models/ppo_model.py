import torch
import torch.nn as nn

class PPOModel(nn.Module):
    def __init__(self, state_dim, action_dim, hidden_dim=256):
        """
        PPO Model combining actor and critic networks.
        Args:
            state_dim (int): Dimension of input state space.
            action_dim (int): Dimension of output action space.
            hidden_dim (int): Number of neurons in hidden layers.
        """
        super(PPOModel, self).__init__()

        # Shared base network
        self.fc1 = nn.Linear(state_dim, hidden_dim)
        self.fc2 = nn.Linear(hidden_dim, hidden_dim)

        # Actor network for policy
        self.actor = nn.Linear(hidden_dim, action_dim)

        # Critic network for value function
        self.critic = nn.Linear(hidden_dim, 1)

    def forward(self, state):
        """
        Forward pass through the shared network.
        Args:
            state (torch.Tensor): Input state tensor.
        Returns:
            torch.Tensor: Processed hidden layer output.
        """
        x = torch.relu(self.fc1(state))
        x = torch.relu(self.fc2(x))
        return x

    def get_action(self, state):
        """
        Get action probabilities for a given state.
        Args:
            state (torch.Tensor): Input state tensor.
        Returns:
            torch.Tensor: Action vector.
        """
        x = self.forward(state)
        action_probs = torch.softmax(self.actor(x), dim=-1)
        action = torch.distributions.Categorical(action_probs).sample().item()  # Discrete action
        return action

    def get_value(self, state):
        """
        Get value estimate for a given state.
        Args:
            state (torch.Tensor): Input state tensor.
        Returns:
            torch.Tensor: Value estimate.
        """
        x = self.forward(state)
        value = self.critic(x)
        return value
