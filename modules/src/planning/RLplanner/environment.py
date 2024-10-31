import numpy as np
import torch
# from sac_agent import SACAgent
from td3_agent import TD3Agent
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
        self.max_action = 10
        self.agent = TD3Agent(self.state_dim, self.action_dim, self.writer)

        self.target_speed = 5

        self.odometry_queue = deque(maxlen=20)
        self.imu_queue = deque(maxlen=20)
        self.objects_queue = deque(maxlen=20)

        self.reset()

    def collision_detected(self):
        self.if_collision = True

    def reset(self):
        self.if_collision = False

    def update_data(self):
        current_speed =  self.odometry_queue[-1].twist.twist.linear.x
        current_acceleration = self.imu_queue[-1].linear_acceleration.x

        distance_to_front_object = 0
        
        orientation_q = self.odometry_queue[-1].pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, yaw = euler_from_quaternion(orientation_list)

        return yaw, distance_to_front_object, current_speed, current_acceleration

    def preprocess_data(self, speed, target_speed):
        state = np.array([speed] + [target_speed])
        return state
    
    def compute_next_state(self, yaw, distance_to_front_object, current_acceleration, current_speed, delta_t=0.1):
        next_speed = current_speed + current_acceleration * delta_t

        next_state = self.preprocess_data(next_speed, self.target_speed)

        return next_state
    
    def compute_reward(self, current_speed):
        speed_reward = -((current_speed - self.target_speed) ** 2)

        total_reward = speed_reward

        return total_reward

    def step(self):
        yaw, distance_to_front_object, current_speed, current_acceleration = self.update_data()

        state = self.preprocess_data(current_speed, self.target_speed)

        action = self.agent.select_action(state, self.step_count)

        next_state = self.compute_next_state(yaw, distance_to_front_object, current_acceleration, current_speed)

        reward = self.compute_reward(current_speed)

        if(self.if_collision):
            print("collision detected!")
            done_bool = True
            self.reset()
        else:
            done_bool = False

        self.agent.memory.add(state, action, next_state, reward, done_bool)

        self.visualize_data(current_speed, current_acceleration, reward)

        self.step_count += 1

        if self.step_count % 10 == 0:
            self.learn()

        scaled_action = (action + 1) * 0.5 * self.max_action

        return done_bool, scaled_action
    
    def learn(self):
        self.agent.train(self.step_count)

    def visualize_data(self, current_speed, current_acceleration, reward):
        self.writer.add_scalar('real speed', current_speed, self.step_count)
        self.writer.add_scalar('real acceleration', current_acceleration, self.step_count)
        self.writer.add_scalar('reward', reward, self.step_count)