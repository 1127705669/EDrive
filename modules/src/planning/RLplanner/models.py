import torch
import torch.nn as nn
import torch.nn.functional as F

class Actor(nn.Module):
    def __init__(self, state_size, action_size):
        super(Actor, self).__init__()
        self.fc1 = nn.Linear(state_size, 400)
        self.fc2 = nn.Linear(400, 300)
        self.fc3 = nn.Linear(300, action_size)
        self.tanh = nn.Tanh()

        # 权重初始化
        self.init_weights()

    def init_weights(self):
        nn.init.uniform_(self.fc1.weight, -0.003, 0.003)
        nn.init.uniform_(self.fc2.weight, -0.003, 0.003)
        nn.init.uniform_(self.fc3.weight, -0.003, 0.003)

    def forward(self, state):
        x = F.relu(self.fc1(state))
        x = F.relu(self.fc2(x))
        x = self.tanh(self.fc3(x))
        return x

class Critic(nn.Module):
    def __init__(self, state_size, action_size):
        super(Critic, self).__init__()
        self.fcs1 = nn.Linear(state_size, 400)
        self.fca1 = nn.Linear(action_size, 300)
        self.fc2 = nn.Linear(400 + 300, 300)  # 修改输入大小以匹配输入特征数量
        self.fc3 = nn.Linear(300, 1)

        # 权重初始化
        self.init_weights()

    def init_weights(self):
        nn.init.uniform_(self.fcs1.weight, -0.003, 0.003)
        nn.init.uniform_(self.fca1.weight, -0.003, 0.003)
        nn.init.uniform_(self.fc2.weight, -0.003, 0.003)
        nn.init.uniform_(self.fc3.weight, -0.003, 0.003)

    def forward(self, state, action):
        s = F.relu(self.fcs1(state))
        a = F.relu(self.fca1(action))
        x = torch.cat((s, a), dim=1)
        x = F.relu(self.fc2(x))
        x = self.fc3(x)
        return x
