import torch
import torch.nn as nn

class PPOModel(nn.Module):
    def __init__(self, state_dim, action_dim, hidden_dim=256):
        super(PPOModel, self).__init__()

        # Shared base network
        self.fc1 = nn.Linear(state_dim, hidden_dim)
        self.fc2 = nn.Linear(hidden_dim, hidden_dim)

        # Separate actor networks for linear and angular velocity
        self.actor_linear = nn.Linear(hidden_dim, 1)  # Linear velocity
        self.actor_angular = nn.Linear(hidden_dim, 1)  # Angular velocity

        # Critic network for value function
        self.critic = nn.Linear(hidden_dim, 1)

    def forward(self, state):
        x = torch.relu(self.fc1(state))
        x = torch.relu(self.fc2(x))
        return x

    def get_action(self, state):
        """
        Get continuous actions for linear and angular velocity.
        Returns tuple of (linear_vel, angular_vel)
        """
        x = self.forward(state)
        
        # Get linear and angular velocities
        linear_vel = torch.tanh(self.actor_linear(x)) * 0.5  # Scale to [-0.5, 0.5]
        angular_vel = torch.tanh(self.actor_angular(x)) * 1.0  # Scale to [-1.0, 1.0]
        
        return (linear_vel.item(), angular_vel.item())

    def get_value(self, state):
        x = self.forward(state)
        value = self.critic(x)
        return value

    def load_state_dict(self, state_dict, strict=True):
        """
        Custom load_state_dict that handles converting old model format to new format
        """
        # If this is an old format state dict, convert it
        if 'actor.weight' in state_dict and 'actor_linear.weight' not in state_dict:
            new_state_dict = {}
            
            # Copy shared layers directly
            for key in ['fc1.weight', 'fc1.bias', 'fc2.weight', 'fc2.bias', 'critic.weight', 'critic.bias']:
                if key in state_dict:
                    new_state_dict[key] = state_dict[key]

            # Convert actor layer to separate linear and angular components
            if 'actor.weight' in state_dict:
                old_actor_weight = state_dict['actor.weight']
                old_actor_bias = state_dict['actor.bias']

                # For 2D action space, split the weights in half
                mid_point = old_actor_weight.size(0) // 2
                
                # Linear velocity components (first half)
                new_state_dict['actor_linear.weight'] = old_actor_weight[0:1]  # Take first row
                new_state_dict['actor_linear.bias'] = old_actor_bias[0:1]      # Take first bias
                
                # Angular velocity components (second half)
                new_state_dict['actor_angular.weight'] = old_actor_weight[1:2]  # Take second row
                new_state_dict['actor_angular.bias'] = old_actor_bias[1:2]      # Take second bias

            # Load the converted state dict
            return super().load_state_dict(new_state_dict, strict=False)
        
        # If it's already in the new format, load normally
        return super().load_state_dict(state_dict, strict=strict)