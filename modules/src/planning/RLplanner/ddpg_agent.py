import torch
import torch.optim as optim
import numpy as np
from collections import deque
from models import Actor, Critic
import random

class DDPGAgent:
    def __init__(self, state_size, action_size, max_action, writer=None):
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        self.actor = Actor(state_size, action_size, max_action).to(self.device)
        self.critic = Critic(state_size, action_size).to(self.device)
        
        self.target_actor = Actor(state_size, action_size, max_action).to(self.device)
        self.target_critic = Critic(state_size, action_size).to(self.device)

        self.actor_optimizer = optim.Adam(self.actor.parameters(), lr=0.001)
        self.critic_optimizer = optim.Adam(self.critic.parameters(), lr=0.002)

        self.max_action = max_action
        self.memory = deque(maxlen=100000)
        self.batch_size = 128
        self.gamma = 0.99
        self.tau = 0.001
        self.noise_scale = 0.01

        # 使用传入的 SummaryWriter
        self.writer = writer
        self.add_graphs_to_tensorboard(self.writer, state_size, action_size)

        # 初始化目标网络
        self.soft_update(self.target_actor, self.actor, 1.0)
        self.soft_update(self.target_critic, self.critic, 1.0)

    def add_graphs_to_tensorboard(self, writer, state_size, action_size):
        if writer is not None:
            # 为actor模型生成虚拟输入并添加图
            dummy_input_actor = torch.randn(1, state_size).float().to(self.device)
            writer.add_graph(self.actor, dummy_input_actor, "Actor")

            # 为target_actor模型生成虚拟输入并添加图
            dummy_input_target_actor = torch.randn(1, state_size).float().to(self.device)
            writer.add_graph(self.target_actor, dummy_input_target_actor, "Target Actor")

            # 为critic模型生成虚拟输入并添加图
            # 注意: critic模型需要状态和动作作为输入
            dummy_state = torch.randn(1, state_size).float().to(self.device)
            dummy_action = torch.randn(1, action_size).float().to(self.device)
            writer.add_graph(self.critic, [dummy_state, dummy_action], "Critic")

            # 为target_critic模型生成虚拟输入并添加图
            writer.add_graph(self.target_critic, [dummy_state, dummy_action], "Target Critic")

    def act(self, state, step):
        state = torch.from_numpy(state).float().to(self.device)
        self.actor.eval()
        with torch.no_grad():
            action = self.actor(state).cpu().data.numpy()
        self.actor.train()
        # 添加噪声进行探索
        action += self.noise_scale * np.random.randn(action.shape[0])
        action = np.clip(action, -self.max_action, self.max_action)

        # 记录动作数据
        if self.writer is not None:
            self.writer.add_scalar('Action/Selected_Action', action[0], step)

        return action

    def remember(self, state, action, reward, next_state, done):
        self.memory.append((state, action, reward, next_state, done))

    def learn(self, step):
        if len(self.memory) < self.batch_size:
            return

        # 随机采样
        experiences = random.sample(self.memory, self.batch_size)

        states, actions, rewards, next_states, dones = zip(*experiences)
        states = torch.tensor(np.vstack(states)).float().to(self.device)
        actions = torch.tensor(np.vstack(actions)).float().to(self.device)
        rewards = torch.tensor(np.vstack(rewards)).float().to(self.device)
        next_states = torch.tensor(np.vstack(next_states)).float().to(self.device)
        dones = torch.tensor(np.vstack(dones).astype(np.uint8)).float().to(self.device)

        # 更新 Critic
        next_actions = self.target_actor(next_states)
        target_q_values = self.target_critic(next_states, next_actions)
        expected_q_values = rewards + (self.gamma * target_q_values * (1 - dones))

        q_values = self.critic(states, actions)
        critic_loss = torch.mean((q_values - expected_q_values.detach()) ** 2)

        self.critic_optimizer.zero_grad()
        critic_loss.backward()
        self.critic_optimizer.step()

        # 记录 Critic 损失
        if self.writer is not None:
            self.writer.add_scalar('Loss/Critic', critic_loss.item(), step)

        # 更新 Actor
        actions_pred = self.actor(states)
        actor_loss = -self.critic(states, actions_pred).mean()

        self.actor_optimizer.zero_grad()
        actor_loss.backward()
        self.actor_optimizer.step()

        # 记录 Actor 损失
        if self.writer is not None:
            self.writer.add_scalar('Loss/Actor', actor_loss.item(), step)

        # 记录权重分布
        if self.writer is not None:
            for name, param in self.actor.named_parameters():
                self.writer.add_histogram(f'Actor/{name}', param, step)
            for name, param in self.critic.named_parameters():
                self.writer.add_histogram(f'Critic/{name}', param, step)

        # 软更新目标网络
        self.soft_update(self.target_actor, self.actor, self.tau)
        self.soft_update(self.target_critic, self.critic, self.tau)

    def soft_update(self, target, source, tau):
        for target_param, param in zip(target.parameters(), source.parameters()):
            target_param.data.copy_(target_param.data * (1.0 - tau) + param.data * tau)

    def save_checkpoint(self):
        torch.save(self.actor.state_dict(), 'actor_checkpoint.pth')
        torch.save(self.critic.state_dict(), 'critic_checkpoint.pth')

    def load_checkpoint(self):
        self.actor.load_state_dict(torch.load('actor_checkpoint.pth', map_location=self.device))
        self.critic.load_state_dict(torch.load('critic_checkpoint.pth', map_location=self.device))
