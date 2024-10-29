import torch
import torch.optim as optim
import torch.nn.functional as F
from models import Actor, Critic
from replay_buffer import ReplayBuffer
import numpy as np
import random

class SACAgent:
    def __init__(self, state_dim, action_dim, hidden_dims=[256, 256], max_action=12, writer=None, alpha=0.2, buffer_size=1000000, batch_size=128, gamma=0.99, tau=0.005):
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.actor = Actor(state_dim, action_dim, hidden_dims, max_action).to(self.device)
        self.critic = Critic(state_dim, action_dim, hidden_dims).to(self.device)
        self.critic_target = Critic(state_dim, action_dim, hidden_dims).to(self.device)
        self.critic_target.load_state_dict(self.critic.state_dict())
        self.actor_optimizer = optim.Adam(self.actor.parameters(), lr=3e-4)
        self.critic_optimizer = optim.Adam(self.critic.parameters(), lr=3e-4)
        self.memory = ReplayBuffer(buffer_size)
        self.batch_size = batch_size
        self.gamma = gamma
        self.tau = tau
        self.alpha = alpha
        self.max_action = max_action

        self.writer = writer
        self.add_graphs_to_tensorboard(self.writer, state_dim, action_dim)

    def add_graphs_to_tensorboard(self, writer, state_size, action_size):
        if writer is not None:
            # 为actor模型生成虚拟输入并添加图
            dummy_input_actor = torch.randn(1, state_size).float().to(self.device)
            writer.add_graph(self.actor, dummy_input_actor, "Actor")

            # 为critic模型生成虚拟输入并添加图
            # 注意: critic模型需要状态和动作作为输入
            dummy_state = torch.randn(1, state_size).float().to(self.device)
            dummy_action = torch.randn(1, action_size).float().to(self.device)
            writer.add_graph(self.critic, [dummy_state, dummy_action], "Critic")

            # 为target_critic模型生成虚拟输入并添加图
            writer.add_graph(self.critic_target, [dummy_state, dummy_action], "Critic Target")

    def select_action(self, state, step, evaluate=False):
        state = torch.FloatTensor(state).unsqueeze(0).to(self.device)
        mean, log_std = self.actor(state)
        std = log_std.exp()

        if evaluate:
            action = mean  # 确定性动作（均值）
        else:
            normal = torch.distributions.Normal(mean, std)
            x_t = normal.rsample()  # 采样
            action = self.max_action * (torch.tanh(x_t) + 1) / 2  # 使用tanh调整并缩放
        
        if self.writer is not None:
            self.writer.add_scalar('Action/Selected_Action', action[0], step)

        return action.cpu().detach().numpy().flatten()

    def update_parameters(self, step):
        if len(self.memory.buffer) < self.batch_size:
            return

        # 从经验回放缓存中随机抽取一个批次的经验
        experiences = random.sample(list(self.memory.buffer), self.batch_size)
        states, actions, rewards, next_states, dones = zip(*experiences)

        # 将数据转换为张量并送到设备
        states = torch.FloatTensor(np.array(states)).to(self.device)
        actions = torch.FloatTensor(np.array(actions)).unsqueeze(-1).to(self.device)
        rewards = torch.FloatTensor(np.array(rewards)).unsqueeze(-1).to(self.device)
        next_states = torch.FloatTensor(np.array(next_states)).to(self.device)
        dones = torch.FloatTensor(np.array(dones)).unsqueeze(-1).to(self.device)

        with torch.no_grad():
            # 生成下一个动作和其对应的log概率
            next_actions, next_log_probs, _ = self.actor.sample(next_states)
            next_log_probs = next_log_probs.unsqueeze(-1)
            q1_next, q2_next = self.critic_target(next_states, next_actions)
            min_q_next = torch.min(q1_next, q2_next) - self.alpha * next_log_probs
            next_q_value = rewards + self.gamma * (1 - dones) * min_q_next

        # 计算当前的Q值
        q1, q2 = self.critic(states, actions)
        critic_loss = F.mse_loss(q1, next_q_value) + F.mse_loss(q2, next_q_value)

        # 更新评论家网络
        self.critic_optimizer.zero_grad()
        critic_loss.backward()
        self.critic_optimizer.step()

        if self.writer is not None:
            self.writer.add_scalar('Loss/Critic', critic_loss.item(), step)

        # 更新演员网络
        next_actions, next_log_probs, _ = self.actor.sample(states)
        next_log_probs = next_log_probs.unsqueeze(-1)
        actor_loss = -(self.critic(states, next_actions)[0] - self.alpha * next_log_probs).mean()  # 使用 self.critic 而不是 self.critic.q1
        self.actor_optimizer.zero_grad()
        actor_loss.backward()
        self.actor_optimizer.step()

        if self.writer is not None:
            self.writer.add_scalar('Loss/Actor', actor_loss.item(), step)

        if self.writer is not None:
            for name, param in self.actor.named_parameters():
                self.writer.add_histogram(f'Actor/{name}', param, step)
            for name, param in self.critic.named_parameters():
                self.writer.add_histogram(f'Critic/{name}', param, step)

        # 更新目标评论家网络
        self.soft_update(self.critic_target, self.critic, self.tau)

    def soft_update(self, target, source, tau):
        for target_param, param in zip(target.parameters(), source.parameters()):
            target_param.data.copy_(tau * param.data + (1 - tau) * target_param.data)
