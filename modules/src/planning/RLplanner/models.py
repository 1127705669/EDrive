import torch
import torch.nn as nn
import torch.nn.functional as F

class Actor(nn.Module):
    def __init__(self, state_size, action_size, max_action):
        super(Actor, self).__init__()

        self.fc1 = nn.Linear(state_size, 400)
        self.bn1 = nn.BatchNorm1d(400)
        self.fc2 = nn.Linear(400, 300)
        self.bn2 = nn.BatchNorm1d(300)
        self.fc3 = nn.Linear(300, action_size)

        self.max_action = max_action

    def forward(self, state):
        x = F.leaky_relu(self.bn1(self.fc1(state)), negative_slope=0.01)
        x = F.leaky_relu(self.bn2(self.fc2(x)), negative_slope=0.01)
        x = self.max_action * torch.sigmoid(self.fc3(x))
        return x

class Critic(nn.Module):
    def __init__(self, state_size, action_size):
        super(Critic, self).__init__()
        
        self.fcs1 = nn.Linear(state_size, 400)
        self.bn1 = nn.BatchNorm1d(400)
        self.fca1 = nn.Linear(action_size, 300)
        self.bn2 = nn.BatchNorm1d(300)
        self.fc2 = nn.Linear(400 + 300, 300)
        self.bn3 = nn.BatchNorm1d(300)
        self.fc3 = nn.Linear(300, 1)

    def forward(self, state, action):
        s = F.leaky_relu(self.bn1(self.fcs1(state)), negative_slope=0.01)
        a = F.leaky_relu(self.bn2(self.fca1(action)), negative_slope=0.01)
        x = torch.cat([s, a], dim=1)
        x = F.leaky_relu(self.bn3(self.fc2(x)), negative_slope=0.01)
        x = self.fc3(x)
        return x
