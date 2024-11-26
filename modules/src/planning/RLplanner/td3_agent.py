import numpy as np
import torch
import torch.optim as optim
import torch.nn.functional as F
from network_td3 import Actor, Critic
from replay_buffer import ReplayBuffer
from ou_noise import OrnsteinUhlenbeckNoise
import random
import copy
import os

class TD3Agent:
    def __init__(self, state_dim, action_dim, writer=None, training_mode=True, fc1_dim=256, fc2_dim=256, buffer_capacity=1000000,
                 batch_size=128, gamma=0.99, tau=0.005, max_action=1.0, policy_noise = 0.1, noise_clip = 0.3, model_dir='model_dir'):
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        self.actor = Actor(state_dim, action_dim, 128, 64).to(self.device)
        self.target_actor = copy.deepcopy(self.actor)
        self.actor_optimizer = optim.Adam(self.actor.parameters(), lr=1e-4)

        self.critic = Critic(state_dim, action_dim, 256, 128).to(self.device)
        self.target_critic = copy.deepcopy(self.critic)
        self.critic_optimizer = optim.Adam(self.critic.parameters(), lr=2e-4, weight_decay=1e-4)

        self.memory = ReplayBuffer(state_dim, action_dim, buffer_capacity)
        self.batch_size = batch_size
        self.gamma = gamma
        self.tau = tau
        self.max_action = max_action
        self.policy_noise = policy_noise
        self.noise_clip = noise_clip

        self.writer = writer
        self.actor_update_flag = 0
        self.training_mode = training_mode

        self.noise = OrnsteinUhlenbeckNoise(dim=(action_dim,), mu=0.0, theta=0.50, sigma=0.30, dt=0.1)

        self.model_dir = model_dir
        if not os.path.exists(self.model_dir):
            os.makedirs(self.model_dir)

        if not training_mode:
            self.load_models()

        self.soft_update(self.target_actor, self.actor, tau=1.0)
        self.soft_update(self.target_critic, self.critic, tau=1.0)

    def soft_update(self, target, source, tau):
        for target_param, param in zip(target.parameters(), source.parameters()):
            target_param.data.copy_(target_param.data * (1.0 - tau) + param.data * tau)

    def save_models(self):
        if self.training_mode:
            torch.save(self.actor.state_dict(), os.path.join(self.model_dir, 'actor.pth'))
            torch.save(self.critic.state_dict(), os.path.join(self.model_dir, 'critic.pth'))
            print("Models saved successfully")

    def load_models(self):
        self.actor.load_state_dict(torch.load(os.path.join(self.model_dir, 'actor.pth'), weights_only=True))
        self.critic.load_state_dict(torch.load(os.path.join(self.model_dir, 'critic.pth'), weights_only=True))
        self.target_actor = copy.deepcopy(self.actor)
        self.target_critic = copy.deepcopy(self.critic)
        if not self.training_mode:
            self.actor.eval()  # Set actor to eval mode if not in training mode
            self.critic.eval()  # Set critic to eval mode if not in training mode
        print("Models loaded successfully")

    def select_action(self, state):
        state = torch.FloatTensor(state.reshape(1, -1)).to(self.device)

        self.actor.eval()

        with torch.no_grad():
            action = self.actor(state).cpu().data.numpy().flatten()

        if self.training_mode:
            self.actor.train()
            action += self.noise()

        action = np.clip(action, -self.max_action, self.max_action)

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
