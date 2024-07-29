# ddpg_agent.py

class DDPG:
    def __init__(self, state_dim, action_dim, max_action):
        self.actor = Actor(state_dim, action_dim, max_action).to(torch.device('cpu'))
        self.actor_target = Actor(state_dim, action_dim, max_action).to(torch.device('cpu'))
        self.actor_target.load_state_dict(self.actor.state_dict())
        self.actor_optimizer = optim.Adam(self.actor.parameters(), lr=LR_ACTOR)

        self.critic = Critic(state_dim, action_dim).to(torch.device('cpu'))
        self.critic_target = Critic(state_dim, action_dim).to(torch.device('cpu'))
        self.critic_target.load_state_dict(self.critic.state_dict())
        self.critic_optimizer = optim.Adam(self.critic.parameters(), lr=LR_CRITIC)

        self.replay_buffer = deque(maxlen=BUFFER_SIZE)
        self.max_action = max_action

    def select_action(self, state):
        state = torch.FloatTensor(state.reshape(1, -1)).to(torch.device('cpu'))
        return self.actor(state).cpu().data.numpy().flatten()

    def train(self, iterations, batch_size=64, gamma=0.99, tau=0.001):
        for _ in range(iterations):
            if len(self.replay_buffer) < batch_size:
                continue

            batch = random.sample(self.replay_buffer, batch_size)
            state, next_state, action, reward, not_done = zip(*batch)

            state = torch.FloatTensor(np.array(state)).to(torch.device('cpu'))
            next_state = torch.FloatTensor(np.array(next_state)).to(torch.device('cpu'))
            action = torch.FloatTensor(np.array(action)).to(torch.device('cpu'))
            reward = torch.FloatTensor(np.array(reward)).to(torch.device('cpu'))
            not_done = torch.FloatTensor(np.array(not_done)).to(torch.device('cpu'))

            next_action = self.actor_target(next_state)
            target_Q = self.critic_target(next_state, next_action)
            target_Q = reward.unsqueeze(1) + not_done.unsqueeze(1) * gamma * target_Q
            current_Q = self.critic(state, action)
            critic_loss = nn.MSELoss()(current_Q, target_Q)

            self.critic_optimizer.zero_grad()
            critic_loss.backward()
            self.critic_optimizer.step()

            actor_loss = -self.critic(state, self.actor(state)).mean()

            self.actor_optimizer.zero_grad()
            actor_loss.backward()
            self.actor_optimizer.step()

            for param, target_param in zip(self.critic.parameters(), self.critic_target.parameters()):
                target_param.data.copy_(tau * param.data + (1 - tau) * target_param.data)

            for param, target_param in zip(self.actor.parameters(), self.actor_target.parameters()):
                target_param.data.copy_(tau * param.data + (1 - tau) * target_param.data)

    def add_to_replay_buffer(self, transition):
        self.replay_buffer.append(transition)

    def predict_speeds(self, states):
        speeds = [self.select_action(np.array(state))[0] for state in states]
        return speeds
