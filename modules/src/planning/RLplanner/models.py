import torch
import torch.nn as nn

class MLP(nn.Module):
    def __init__(self, input_dim, hidden_dims, output_dim, activation=nn.LeakyReLU):
        super(MLP, self).__init__()
        layers = []
        dims = [input_dim] + hidden_dims
        for i in range(len(dims) - 1):
            layers.append(nn.Linear(dims[i], dims[i + 1]))
            layers.append(nn.LayerNorm(dims[i + 1]))  # 添加层归一化
            layers.append(activation(negative_slope=0.01))  # LeakyReLU激活函数
        layers.append(nn.Linear(dims[-1], output_dim))
        self.model = nn.Sequential(*layers)

    def forward(self, x):
        return self.model(x)

class Actor(nn.Module):
    def __init__(self, state_dim, action_dim, hidden_dims, max_action, log_std_min=-20, log_std_max=2):
        super(Actor, self).__init__()
        self.max_action = max_action
        self.log_std_min = log_std_min
        self.log_std_max = log_std_max
        self.base = MLP(state_dim, hidden_dims, 2 * action_dim)

    def forward(self, state):
        mean_log_std = self.base(state)
        mean, log_std = torch.chunk(mean_log_std, 2, dim=-1)
        log_std = torch.clamp(log_std, self.log_std_min, self.log_std_max)
        std = log_std.exp()
        mean = self.max_action * (torch.tanh(mean) + 1) / 2  # 使用tanh调整并缩放到[0, max_action]
        return mean, std

    def sample(self, state):
        mean, std = self.forward(state)
        normal = torch.distributions.Normal(mean, std)
        x_t = normal.rsample()
        action = self.max_action * (torch.tanh(x_t) + 1) / 2  # 使用tanh调整并缩放
        log_prob = normal.log_prob(x_t) - torch.log((1 - torch.tanh(x_t).pow(2)) + 1e-6)
        return action, log_prob.sum(dim=-1), mean

class Critic(nn.Module):
    def __init__(self, state_dim, action_dim, hidden_dims):
        super(Critic, self).__init__()
        self.q1 = MLP(state_dim + action_dim, hidden_dims, 1)
        self.q2 = MLP(state_dim + action_dim, hidden_dims, 1)

    def forward(self, state, action):
        sa = torch.cat([state, action], dim=-1)
        q1 = self.q1(sa)
        q2 = self.q2(sa)
        return q1, q2
