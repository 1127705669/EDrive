class Environment:
    def __init__(self, target_speed):
        self.target_speed = target_speed
        self.states = []
        self.previous_speeds = [0] * 50  # 假设有50个点
        self.jerks = [0] * 50

    def calculate_reward(self, current_speed, expected_speed):
        # 计算速度误差
        speed_error = abs(current_speed - expected_speed)
        # 计算加加速度
        jerk = abs(current_speed - self.previous_speeds)
        # 计算奖励
        reward = -speed_error - 0.1 * jerk
        return reward

    def reset(self, initial_states):
        self.states = initial_states
        self.previous_speeds = [0] * 50
        self.jerks = [0] * 50
        return self.states

    def update_current_state(self, index, current_x, current_y, current_speed):
        # 确保 index 在 states 列表的范围内
        if index >= len(self.states):
            self.states.append([current_x, current_y, current_speed])
        else:
            self.previous_speeds[index] = self.states[index][2]
            self.states[index] = [current_x, current_y, current_speed]

    def step(self, expected_speeds):
        # 假设 expected_speeds 是一个包含所有点期望速度的列表
        rewards = []
        next_states = []
        dones = []

        for i, expected_speed in enumerate(expected_speeds):
            current_x, current_y, current_speed = self.states[i]

            # 更新速度
            self.previous_speeds[i] = current_speed
            current_speed = expected_speed

            # 更新状态
            self.states[i] = [current_x, current_y, current_speed]
            self.jerks[i] = abs(current_speed - self.previous_speeds[i])
            next_states.append(self.states[i])

            # 计算奖励
            reward = self.calculate_reward(current_speed, expected_speed)
            rewards.append(reward)

            # 判断终止条件 (假设速度超过一定范围即终止)
            done = current_speed > 30 or current_speed < -30
            dones.append(done)

        return next_states, rewards, dones
