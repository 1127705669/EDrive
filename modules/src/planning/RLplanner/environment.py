# environment.py

class Environment:
    def __init__(self, target_speed):
        self.target_speed = target_speed
        self.current_speed = 0
        self.previous_speed = 0
        self.jerk = 0

    def calculate_reward(self, current_speed):
        # 计算速度误差
        speed_error = abs(current_speed - self.target_speed)
        # 计算加加速度
        jerk = abs(current_speed - self.previous_speed)
        # 计算奖励
        reward = -speed_error - 0.1 * jerk
        return reward

    def reset(self):
        self.current_speed = 0
        self.previous_speed = 0
        self.jerk = 0
        return self.current_speed, self.jerk

    def step(self, action):
        self.previous_speed = self.current_speed
        self.current_speed = action  # 假设action是速度
        reward = self.calculate_reward(self.current_speed)
        state = (self.current_speed, self.jerk)
        done = False  # 假设任务永远不会完成
        return state, reward, done
