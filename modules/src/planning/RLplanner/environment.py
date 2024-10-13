import numpy as np
from ddpg_agent import DDPGAgent
from collections import deque
from tf.transformations import euler_from_quaternion
from torch.utils.tensorboard import SummaryWriter

class Environment:
    def __init__(self, max_action, target_speed, writer=None):
        # 初始化强化学习代理
        if writer is None:
            self.writer = SummaryWriter('runs/ddpg_training')  # 如果没有传入 writer，就创建一个默认的
        else:
            self.writer = writer

        self.state_size = 81
        self.action_size = 1
        self.max_action = max_action

        self.agent = DDPGAgent(self.state_size, self.action_size, self.max_action, writer=self.writer)
        self.target_speed = target_speed  # 目标速度
        self.state = self.reset()
        self.done = False
        self.distance = 0
        self.speed = 0
        self.acceleration = 0

        # 存储传感器原始数据
        self.odometry_queue = deque(maxlen=20)
        self.imu_queue = deque(maxlen=20)

        # 存储经验
        self.memory = deque(maxlen=10000)

    def reset(self):
        # 重置环境到初始状态
        self.distance = 0  # 从起点到当前位置的距离
        self.speed = 0  # 当前速度
        self.acceleration = 0  # 初始加速度为0
        self.done = False

        # 构造初始状态向量 (20 个速度为 0, 20 个 x 位置为 0, 20 个 y 位置为 0, 20 个加速度为 0, 目标速度为 target_speed)
        speeds = [0] * 20
        positions_x = [0] * 20
        positions_y = [0] * 20
        accelerations = [0] * 20
        state = np.array(speeds + positions_x + positions_y + accelerations + [self.target_speed])  # 81 维状态向量

        self.state = state
        return self.state

    def update_data(self, odometry_queue, imu_queue):
        """
        更新环境内部的传感器数据队列。
        
        参数：
        odometry_queue: 完整的odometry数据队列
        imu_queue: 完整的IMU数据队列
        """
        # 更新内部的odometry_queue和imu_queue，确保与ROSNode中保持同步from torch.utils.tensorboard import SummaryWriter
        self.odometry_queue.clear()
        self.imu_queue.clear()

        self.odometry_queue.extend(odometry_queue)
        self.imu_queue.extend(imu_queue)

    def preprocess_data(self,odometry_queue, imu_queue):
        """
        对存储的原始数据进行预处理，用于强化学习的输入。
        
        返回：
        state (np.array): 预处理后的状态向量
        """
        # 从odometry队列提取速度和位置
        speeds = [data.twist.twist.linear.x for data in list(odometry_queue)]
        positions_x = [data.pose.pose.position.x for data in list(odometry_queue)]
        positions_y = [data.pose.pose.position.y for data in list(odometry_queue)]
        
        # 从IMU队列提取加速度
        accelerations = [data.linear_acceleration.x for data in list(imu_queue)]

        # 构造状态向量
        state = np.array(speeds + positions_x + positions_y + accelerations + [self.target_speed])
        return state

    def step(self, step_count):
        """
        执行一步环境更新，并获取下一状态和奖励。
        
        返回：
        next_state (np.array): 更新后的下一状态
        reward (float): 当前状态的奖励
        done (bool): 指示是否结束
        """

        # 预处理数据以获取当前状态
        current_state = self.preprocess_data(self.odometry_queue, self.imu_queue)

        # 使用RL模型基于当前状态做出决策
        action = self.agent.act(current_state, step_count)  # 动作是目标速度，RL的输出为目标速度
        target_speed = action[0]  # 取出目标速度

        imu_queue_copy = deque(self.imu_queue, maxlen=20)  # 复制 IMU 队列

        # 从odometry数据获取航向角
        orientation_q = self.odometry_queue[-1].pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, yaw = euler_from_quaternion(orientation_list)  # 获取航向角（偏航角）

        # 将车辆坐标系中的速度转换到全局坐标系
        # 更新速度以逐步接近目标速度
        self.acceleration = imu_queue_copy[-1].linear_acceleration.x
        self.speed = self.odometry_queue[-1].twist.twist.linear.x + self.acceleration * 0.05  # 更新速度
        current_speed_local = self.speed
        speed_global_x = current_speed_local * np.cos(yaw)
        speed_global_y = current_speed_local * np.sin(yaw)

        # 推算下一时刻的位置
        current_position_x = self.odometry_queue[-1].pose.pose.position.x
        current_position_y = self.odometry_queue[-1].pose.pose.position.y
        delta_t = 0.1  # 时间步长，单位秒

        new_position_x = current_position_x + speed_global_x * delta_t
        new_position_y = current_position_y + speed_global_y * delta_t

        # 创建新的 odometry 数据对象，并添加到副本队列中
        new_odometry_data = self.odometry_queue[-1]  # 复制最后一帧数据
        new_odometry_data.twist.twist.linear.x = current_speed_local  # 保持局部速度不变
        new_odometry_data.pose.pose.position.x = new_position_x
        new_odometry_data.pose.pose.position.y = new_position_y

        # 使用副本队列构造下一时刻的状态
        odometry_queue_copy = deque(self.odometry_queue, maxlen=20)
        odometry_queue_copy.append(new_odometry_data)

        # 使用副本数据预处理生成 next_state
        next_state = self.preprocess_data(odometry_queue_copy, imu_queue_copy)

        # 计算奖励
        speed_error = np.abs(current_speed_local - self.target_speed)
        reward = -speed_error  # 奖励是负的速度误差

        print(f"reward: {reward}, current_speed: {current_speed_local}, target_speed: {self.target_speed}")

        # 检查是否结束（假设距离达到 10km 时结束）
        self.done = self.distance >= 1000000

        # 将经验存储到内存中 (state, action, reward, next_state, done)
        self.memory.append((current_state, action, reward, next_state, self.done))

        # 存储当前状态用于 TensorBoard 可视化
        self.writer.add_scalar('Speed', current_speed_local, step_count)
        self.writer.add_scalar('Acceleration', self.acceleration, step_count)
        self.writer.add_scalar('Target_Speed', target_speed, step_count)
        self.writer.add_scalar('Reward', reward, step_count)

        return next_state, reward, self.done, target_speed

    def render(self):
        # 可选：提供一种可视化当前环境状态的方式
        print(f"Distance: {self.distance} m, Speed: {self.speed} m/s, Acceleration: {self.acceleration} m/s²")

    def get_stored_data(self):
        """
        获取存储的数据，用于分析和调试
        """
        return list(self.memory)
