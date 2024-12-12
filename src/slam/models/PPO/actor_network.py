import torch as T
import torch.nn as nn
import torch.optim as optim
import os

class ActorNetwork(nn.Module):
    # def __init__(self, n_actions, input_dims, alpha, fc1_dims = 256, fc2_dims = 256, chkpt_dir = 'tmp/ppo'):
    #     super(ActorNetwork, self).__init__()

    #     self.checkpoint_file = os.path.join(chkpt_dir, 'actor_torch_ppo')

    #     self.actor = nn.Sequential(
    #         nn.Linear(*input_dims, fc1_dims),
    #         nn.ReLU(),
    #         nn.Linear(fc1_dims, fc2_dims),
    #         nn.ReLU(),
    #         nn.Linear(fc2_dims, n_actions),
    #         nn.Softmax(dim=-1)
    #     )

    #     self.optimizer = optim.Adam(self.parameters(), lr=alpha)
    #     self.device = T.device('cuda' if T.cuda.is_available() else 'cpu')
    #     self.to(self.device)

    # def forward(self, state):
    #     dist = self.actor(state)
    #     dist = Categorical(dist)

    #     return dist

    def __init__(self, n_actions, input_dims, alpha, fc1_dims = 256, fc2_dims = 256, chkpt_dir = 'tmp/ppo'):
        super(ActorNetwork, self).__init__()

        self.checkpoint_file = os.path.join(chkpt_dir, 'actor_torch_ppo')

        self.fc1 = nn.Linear(*input_dims, fc1_dims)
        self.fc2 = nn.Linear(fc1_dims, fc2_dims)
        self.mean_layer = nn.Linear(fc2_dims, n_actions)  # Outputs the mean of the actions
        self.log_std_layer = nn.Linear(fc2_dims, n_actions)  # Outputs the log std of the actions

        self.optimizer = optim.Adam(self.parameters(), lr=alpha)
        self.device = T.device('cuda' if T.cuda.is_available() else 'cpu')
        self.to(self.device)

    def forward(self, state):
        x = nn.functional.relu(self.fc1(state))
        x = nn.functional.relu(self.fc2(x))
        mean = T.tanh(self.mean_layer(x))  # Ensure actions are within [-1, 1] (optional for bounded actions)
        log_std = T.clamp(self.log_std_layer(x), min=-20, max=2)  # Log std for stability
        std = T.exp(log_std)  # Convert log_std to std
        
        dist = T.distributions.Normal(mean, std)

        return dist
    
    def save_checkpoint(self):
        T.save(self.state_dict(), self.checkpoint_file)

    def load_checkpoint(self):
        self.load_state_dict(T.load(self.checkpoint_file))