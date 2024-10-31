import torch
import torch.nn as nn
import torch.nn.init as init

class MLP(nn.Module):
    def __init__(self, input_dim, hidden_dims, output_dim, activation=nn.LeakyReLU):
        super(MLP, self).__init__()
        layers = []
        dims = [input_dim] + hidden_dims
        for i in range(len(dims) - 1):
            linear = nn.Linear(dims[i], dims[i + 1])
            init.xavier_uniform_(linear.weight)
            layers.append(linear)
            layers.append(nn.LayerNorm(dims[i + 1]))
            layers.append(activation(negative_slope=0.01))
        linear = nn.Linear(dims[-1], output_dim)
        init.xavier_uniform_(linear.weight)
        layers.append(linear)
        self.model = nn.Sequential(*layers)

    def forward(self, x):
        return self.model(x)

class Actor(nn.Module):
    def __init__(self, state_dim, action_dim, hidden_dims, log_std_min=-20, log_std_max=2):
        super(Actor, self).__init__()
        self.log_std_min = log_std_min
        self.log_std_max = log_std_max
        self.base = MLP(state_dim, hidden_dims, 2 * action_dim)

    def forward(self, state, training=True):
        mean_log_std = self.base(state)
        mean, log_std = torch.chunk(mean_log_std, 2, dim=-1)
        if training:
            log_std = torch.clamp(log_std, self.log_std_min, self.log_std_max)  # 训练时裁剪
        std = log_std.exp()
        mean = torch.tanh(mean)
        return mean, std

    def sample(self, state, training=True):
        mean, std = self.forward(state, training)
        normal = torch.distributions.Normal(mean, std)
        x_t = normal.rsample()
        action = torch.tanh(x_t)
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
