import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F


class Actor(nn.Module):
    def __init__(self, state_dim, action_dim, fc1_dim, fc2_dim):
        super(Actor, self).__init__()

        self.fc1 = nn.Linear(state_dim, fc1_dim)
        self.ln1 = nn.LayerNorm(fc1_dim)
        self.fc2 = nn.Linear(fc1_dim, fc2_dim)
        self.ln2 = nn.LayerNorm(fc2_dim)
        self.action = nn.Linear(fc2_dim, action_dim)

    def forward(self, state):
        x = torch.relu(self.ln1(self.fc1(state)))
        x = torch.relu(self.ln2(self.fc2(x)))
        action = torch.tanh(self.action(x))

        return action


class Critic(nn.Module):
    def __init__(self, state_dim, action_dim, fc1_dim, fc2_dim):
        super(Critic, self).__init__()

        # Q1 architecture
        self.l1 = nn.Linear(state_dim + action_dim, fc1_dim)
        self.ln1 = nn.LayerNorm(fc1_dim)
        self.l2 = nn.Linear(fc1_dim, fc2_dim)
        self.ln2 = nn.LayerNorm(fc2_dim)
        self.q1 = nn.Linear(fc2_dim, 1)

        # Q2 architecture
        self.l4 = nn.Linear(state_dim + action_dim, fc1_dim)
        self.ln4 = nn.LayerNorm(fc1_dim)
        self.l5 = nn.Linear(fc1_dim, fc2_dim)
        self.ln5 = nn.LayerNorm(fc2_dim)
        self.q2 = nn.Linear(fc2_dim, 1)


    def forward(self, state, action):
        sa = torch.cat([state, action], dim=1)

        # Path for Q1
        x1 = F.relu(self.ln1(self.l1(sa)))
        x1 = F.relu(self.ln2(self.l2(x1)))
        q1 = self.q1(x1)

        # Path for Q2
        x2 = F.relu(self.ln4(self.l4(sa)))
        x2 = F.relu(self.ln5(self.l5(x2)))
        q2 = self.q2(x2)

        return q1, q2

    def Q1(self, state, action):
        sa = torch.cat([state, action], dim=1)
        x1 = F.relu(self.ln1(self.l1(sa)))
        x1 = F.relu(self.ln2(self.l2(x1)))
        q1 = self.q1(x1)
        return q1
