import numpy as np
import torch
from sac_agent import SACAgent
from collections import deque
from tf.transformations import euler_from_quaternion
from torch.utils.tensorboard import SummaryWriter
import math

class Environment:
    def __init__(self):
        self.step_count = 0
        self.writer = SummaryWriter('runs/sac_training')
        self.state_dim = 2
        self.action_dim = 1
        self.max_action = 12
        self.agent = SACAgent(self.state_dim, self.action_dim, [128, 256], self.max_action, self.writer)

        self.distance_to_front_object = 0
        self.target_speed = 5
        self.done = False
        self.distance = 0
        self.speed = 0
        self.acceleration = 0
        self.yaw = 0
        self.position_x = 0
        self.position_y = 0
        self.if_collision = False

        self.state = self.reset()
        self.odometry_queue = deque(maxlen=20)
        self.imu_queue = deque(maxlen=20)
        self.objects_queue = deque(maxlen=20)
        self.collision_queue = deque(maxlen=20)

    def collision_detected(self):
        self.if_collision = True

    def reset(self):
        # 重置环境到初始状态
        self.distance = 0  # 从起点到当前位置的距离
        self.speed = 0  # 当前速度
        self.acceleration = 0  # 初始加速度为0
        self.done = False
        self.if_collision = False

        state = np.array([self.speed] + [self.target_speed])

        return state.reshape(1, -1)

    def update_data(self, odometry_queue, imu_queue, objects_queue, collision_queue):
        """
        更新环境内部的传感器数据队列。
        
        参数：
        odometry_queue: 完整的odometry数据队列
        imu_queue: 完整的IMU数据队列
        """
        # 更新内部的odometry_queue和imu_queue，确保与ROSNode中保持同步from torch.utils.tensorboard import SummaryWriter
        self.odometry_queue.clear()
        self.imu_queue.clear()
        self.objects_queue.clear()
        self.collision_queue.clear()

        self.odometry_queue.extend(odometry_queue)
        self.imu_queue.extend(imu_queue)
        self.objects_queue.extend(objects_queue)
        self.collision_queue.extend(collision_queue)

        self.position_x = odometry_queue[-1].pose.pose.position.x
        self.position_y = odometry_queue[-1].pose.pose.position.y

        latest_objects = self.objects_queue[-1]

        # position = latest_objects.objects[0].pose.position
        # object_x = position.x
        # object_y = position.y

        # # 计算自车与目标车辆的欧几里得距离
        # self.distance_to_front_object = math.sqrt((self.position_x - object_x)**2 + (self.position_y - object_y)**2)

        # if(self.distance_to_front_object > 15):
        #     self.distance_to_front_object = 0

        # print(self.distance_to_front_object)

        # 从odometry数据获取航向角
        orientation_q = self.odometry_queue[-1].pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, self.yaw = euler_from_quaternion(orientation_list)  # 获取航向角（偏航角）

    def preprocess_data(self,odometry_queue, imu_queue, next_distance):
        """
        对存储的原始数据进行预处理，用于强化学习的输入。
        
        返回：
        state (np.array): 预处理后的状态向量
        """
        # 从odometry队列提取速度和位置
        speed = odometry_queue[-1].twist.twist.linear.x
        position_x = odometry_queue[-1].pose.pose.position.x
        position_y = odometry_queue[-1].pose.pose.position.y

        if_collision = self.if_collision

        # 构造状态向量
        state = np.array([speed] + [self.target_speed])
        
        return state
    
    def compute_next_state(self, delta_t=0.1):
        """
        计算下一状态，根据目标速度和当前状态推算下一时刻的状态。
        
        参数：
        target_speed (float): RL输出的目标速度。
        delta_t (float): 时间步长，单位秒。

        返回：
        next_state (np.array): 更新后的下一状态
        odometry_queue_copy (deque): 更新后的odometry队列副本
        imu_queue_copy (deque): 更新后的IMU队列副本
        """

        # 复制 IMU 队列
        imu_queue_copy = deque(self.imu_queue, maxlen=20)

        # 将车辆坐标系中的速度转换到全局坐标系
        # 更新速度以逐步接近目标速度
        self.acceleration = imu_queue_copy[-1].linear_acceleration.x
        self.speed = self.odometry_queue[-1].twist.twist.linear.x + self.acceleration * 0.05  # 更新速度
        current_speed_local = self.speed
        speed_global_x = current_speed_local * np.cos(self.yaw)
        speed_global_y = current_speed_local * np.sin(self.yaw)

        # 推算下一时刻的位置
        current_position_x = self.odometry_queue[-1].pose.pose.position.x
        current_position_y = self.odometry_queue[-1].pose.pose.position.y

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

        if(self.distance_to_front_object > 15):
            next_distance = 0
        else:
            next_distance = self.distance_to_front_object - delta_t*speed_global_x

        # 使用副本数据预处理生成 next_state
        next_state = self.preprocess_data(odometry_queue_copy, imu_queue_copy, next_distance)

        return next_state, odometry_queue_copy, imu_queue_copy
    
    def compute_reward(self, current_speed):
        # 计算奖励

        # 参数设置
        weight_speed = 0.75  # 速度权重
        weight_distance = 0.25  # 距离权重
        best_distance = 6  # 理想距离
        distance_threshold = 15  # 距离阈值
        lower_threshold = 6  # 下限阈值
        k = 10  # 最大奖励

        vehicle_reset = False

        if(self.if_collision):
            print("collision detected!")
            vehicle_reset = True
            self.if_collision = False

        # 速度奖励：权重a乘以（当前速度-目标车速）的平方
        
        speed_reward = np.exp(-0.5 * ((current_speed - self.target_speed) ** 2) / (1 ** 2))

        # 距离奖励
        if 4 > self.distance_to_front_object or self.distance_to_front_object > distance_threshold:
            distance_reward = 0  # 距离大于15米时奖励为0
        else:
            # 在15米到6米之间，奖励逐渐增加，使用一个简单的线性关系
            distance_reward = -weight_distance * (distance_threshold - self.distance_to_front_object)**2

        total_reward = speed_reward #+ distance_reward

        # print("reward: " + str(total_reward) + ", distance reward: " + str(distance_reward) + ", speed reward: " + str(speed_reward))

        # print(f"reward: {reward}, current_speed: {current_speed}, target_speed: {self.target_speed}")

        return total_reward, vehicle_reset

    def step(self):
        """
        执行一步环境更新，并获取下一状态和奖励。
        
        返回：
        next_state (np.array): 更新后的下一状态
        reward (float): 当前状态的奖励
        done (bool): 指示是否结束
        """

        # 预处理数据以获取当前状态
        current_state = self.preprocess_data(self.odometry_queue, self.imu_queue, self.distance_to_front_object)

        # 使用RL模型基于当前状态做出决策
        action = self.agent.select_action(current_state, self.step_count)  # 动作是目标速度，RL的输出为目标速度
        action = action[0]

        print(action)

        next_state, odometry_queue_copy, imu_queue_copy = self.compute_next_state(delta_t=0.1)

        reward, vehicle_reset = self.compute_reward(self.odometry_queue[-1].twist.twist.linear.x)

        # 检查是否结束（假设距离达到 10km 时结束）
        self.done = self.distance >= 1000000

        # 将经验存储到内存中 (state, action, reward, next_state, done)
        self.agent.memory.push(current_state, action, reward, next_state, self.done)

        # 存储当前状态用于 TensorBoard 可视化
        self.writer.add_scalar('real speed', self.odometry_queue[-1].twist.twist.linear.x, self.step_count)
        self.writer.add_scalar('real acceleration', self.acceleration, self.step_count)
        self.writer.add_scalar('reward', reward, self.step_count)

        self.step_count += 1

        if self.step_count % 10 == 0:
            learn_flag = True
        else:
            learn_flag = False

        return next_state, reward, self.done, action, vehicle_reset, learn_flag
    
    def format_data(self, data, dim):
        """
        保证数据具有正确的形状，用于存储和处理。
        参数：
        data: 要格式化的数据
        dim: 数据应有的维度
        """
        return np.array(data, dtype=np.float32).reshape(1, dim)
    
    def learn(self):
        self.agent.update_parameters(self.step_count)

    def render(self):
        # 可选：提供一种可视化当前环境状态的方式
        print(f"Distance: {self.distance} m, Speed: {self.speed} m/s, Acceleration: {self.acceleration} m/s²")

    def get_stored_data(self):
        """
        获取存储的数据，用于分析和调试
        """
        return list(self.agent.memory)
