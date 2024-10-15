import torch
import torch.nn as nn
import torch.nn.functional as F

class Actor(nn.Module):
    def __init__(self, state_size, action_size, max_action):
        super(Actor, self).__init__()

        self.fc1 = nn.Linear(state_size, 400)
        self.fc2 = nn.Linear(400, 300)
        self.fc3 = nn.Linear(300, action_size)
        self.sigmoid = nn.Sigmoid()

        self.max_action = max_action

    def forward(self, state):
        x = F.relu(self.fc1(state))
        x = F.relu(self.fc2(x))
        x = self.max_action * self.sigmoid(self.fc3(x))
        return x

class Critic(nn.Module):
    def __init__(self, state_size, action_size):
        super(Critic, self).__init__()
        
        self.fcs1 = nn.Linear(state_size, 400)
        self.fca1 = nn.Linear(action_size, 300)
        self.fc2 = nn.Linear(400 + 300, 300)
        self.fc3 = nn.Linear(300, 1)

    def forward(self, state, action):
        s = F.relu(self.fcs1(state))
        a = F.relu(self.fca1(action))
        x = torch.cat([s, a], dim=1)
        x = F.relu(self.fc2(x))
        x = self.fc3(x)
        return x
