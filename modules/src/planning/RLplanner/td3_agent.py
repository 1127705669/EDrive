import numpy as np
import torch
import torch.optim as optim
import torch.nn.functional as F
from network_td3 import Actor, Critic
from replay_buffer import ReplayBuffer
from ou_noise import OrnsteinUhlenbeckNoise
import random
import copy

class TD3Agent:
    def __init__(self, state_dim, action_dim, writer=None, raining_mode=True, fc1_dim=256, fc2_dim=256, buffer_capacity=1000000,
                 batch_size=256, gamma=0.99, tau=0.005, max_action=1.0, policy_noise = 0.2, noise_clip = 0.5):
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        self.actor = Actor(state_dim, action_dim, 128, 64).to(self.device)
        self.target_actor = copy.deepcopy(self.actor)
        self.actor_optimizer = optim.Adam(self.actor.parameters(), lr=1e-4)

        self.critic = Critic(state_dim, action_dim, 256, 128).to(self.device)
        self.target_critic = copy.deepcopy(self.critic)
        self.critic_optimizer = optim.Adam(self.critic.parameters(), lr=1e-4)

        self.memory = ReplayBuffer(state_dim, action_dim, buffer_capacity)
        self.batch_size = batch_size
        self.gamma = gamma
        self.tau = tau
        self.max_action = max_action
        self.policy_noise = policy_noise
        self.noise_clip = noise_clip

        self.writer = writer
        self.actor_update_flag = 0

        self.noise = OrnsteinUhlenbeckNoise(dim=(action_dim,), mu=0.0, theta=0.01, sigma=0.01, dt=0.1)

        self.soft_update(self.target_actor, self.actor, tau=1.0)
        self.soft_update(self.target_critic, self.critic, tau=1.0)

    def soft_update(self, target, source, tau):
        for target_param, param in zip(target.parameters(), source.parameters()):
            target_param.data.copy_(target_param.data * (1.0 - tau) + param.data * tau)

    def select_action(self, state, step):
        # 将状态转换为PyTorch张量，并添加批次维度
        state = torch.FloatTensor(state.reshape(1, -1)).to(self.device)

        # 设置网络为评估模式
        self.actor.eval()

        # 禁用梯度计算
        with torch.no_grad():
            action = self.actor(state).cpu().data.numpy().flatten()

        # 恢复训练模式
        self.actor.train()

        # 添加噪声来进行探索
        action += self.noise()

        # 限制动作在允许的范围内
        action = np.clip(action, -1.0, 1.0)

        # 记录动作数据到 TensorBoard，如果配置了的话
        self.writer.add_scalar('Action/Selected_Action', action, step)

        return action

    def train(self, step):
        if self.memory.ptr < self.batch_size:
            return

        # Sample replay buffer 
        state, action, next_state, reward, not_done = self.memory.sample(self.batch_size)

        with torch.no_grad():
            # Select action according to policy and add clipped noise
            noise = (
                torch.randn_like(action) * self.policy_noise
            ).clamp(-self.noise_clip, self.noise_clip)

            next_action = (
                self.target_actor(next_state) + noise
            ).clamp(-self.max_action, self.max_action)
            
            # Compute the target Q value
            target_Q1, target_Q2 = self.target_critic(next_state, next_action)
            target_Q = torch.min(target_Q1, target_Q2)
            target_Q = reward + not_done * self.gamma * target_Q

        # Get current Q estimates
        current_Q1, current_Q2 = self.critic(state, action)

        # Compute critic loss
        critic_loss = F.mse_loss(current_Q1, target_Q) + F.mse_loss(current_Q2, target_Q)

        self.writer.add_scalar('Loss/Critic', critic_loss.item(), step)

        for name, param in self.critic.named_parameters():
                self.writer.add_histogram(f'Critic/{name}', param, step)

        # Optimize the critic
        self.critic_optimizer.zero_grad()
        critic_loss.backward()
        self.critic_optimizer.step()

        # 更新Actor每两个训练步骤更新一次
        self.actor_update_flag += 1
        if self.actor_update_flag == 2:
            self.actor_update_flag = 0

            # Compute actor losse
            actor_loss = -self.critic.Q1(state, self.actor(state)).mean()

            # Optimize the actor 
            self.actor_optimizer.zero_grad()
            actor_loss.backward()
            self.actor_optimizer.step()
            
            # 更新目标Actor网络
            self.soft_update(self.target_critic, self.critic, self.tau)
            self.soft_update(self.target_actor, self.actor, self.tau)

            self.writer.add_scalar('Loss/Actor', actor_loss.item(), step)

            for name, param in self.actor.named_parameters():
                self.writer.add_histogram(f'Actor/{name}', param, step)
