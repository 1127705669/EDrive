import numpy as np
import torch
# from sac_agent import SACAgent
from td3_agent import TD3Agent
from collections import deque
from tf.transformations import euler_from_quaternion
from torch.utils.tensorboard import SummaryWriter
import math
import time


class Environment:
    def __init__(self):
        self.step_count = 0
        self.writer = SummaryWriter('runs/td3')
        self.state_dim = 2
        self.action_dim = 1
        self.max_action = 10
        self.agent = TD3Agent(self.state_dim, self.action_dim, self.writer)

        self.target_speed = 5

        self.odometry_queue = deque(maxlen=20)
        self.imu_queue = deque(maxlen=20)
        self.objects_queue = deque(maxlen=20)

        self.init_position_recored = False
        self.ego_posotion_x = 0
        self.ego_posotion_y = 0

        self.is_left = False

        self.reset()

    def collision_detected(self):
        self.if_collision = True
        self.last_coliision = self.if_collision

    def reset(self):
        self.if_collision = False
        self.last_coliision = self.if_collision
        self.init_position_recored = False
        self.is_left = False


    def update_data(self):
        position_x = self.odometry_queue[-1].pose.pose.position.x
        position_y = self.odometry_queue[-1].pose.pose.position.y

        yaw = euler_from_quaternion([
            self.odometry_queue[-1].pose.pose.orientation.x,
            self.odometry_queue[-1].pose.pose.orientation.y,
            self.odometry_queue[-1].pose.pose.orientation.z,
            self.odometry_queue[-1].pose.pose.orientation.w
        ])[-1]

        speed = self.odometry_queue[-1].twist.twist.linear.x
        ego_speed_x = self.odometry_queue[-1].twist.twist.linear.x * np.cos(yaw)
        ego_speed_y = self.odometry_queue[-1].twist.twist.linear.x * np.sin(yaw)
        acceleration = self.imu_queue[-1].linear_acceleration.x
        acceleration_x = self.imu_queue[-1].linear_acceleration.x * np.cos(yaw)
        acceleration_y = self.imu_queue[-1].linear_acceleration.x * np.sin(yaw)
        angular_velocity = self.odometry_queue[-1].twist.twist.angular.z
        angular_acceleration = self.imu_queue[-1].angular_velocity.z

        # print('-------------------- Ego Vehicle --------------------')
        # print(f"Position: ({position_x}, {position_y})")
        # print(f"Yaw: {yaw}")
        # print(f"Speed (in x, y components): ({ego_speed_x}, {ego_speed_y})")
        # print(f"Acceleration: {acceleration}")
        # print(f"Acceleration (in x, y components): ({acceleration_x}, {acceleration_y})")
        # print(f"Angular Velocity: {angular_velocity}")
        # print(f"Angular Acceleration: {angular_acceleration}")
        # print('------------------------------------------------------')

        ego_vehicle = {
            'position_x': position_x,
            'position_y': position_y,
            'yaw': yaw,
            'speed': speed,
            'acceleration': acceleration,
            'angular_velocity': angular_velocity,
            'angular_acceleration': angular_acceleration
        }

        objects = []
        for obj in self.objects_queue[-1].objects:
            absolute_position_x = obj.pose.position.x
            absolute_position_y = obj.pose.position.y
            absolute_yaw = euler_from_quaternion([
                obj.pose.orientation.x,
                obj.pose.orientation.y,
                obj.pose.orientation.z,
                obj.pose.orientation.w
            ])[-1]
            obj_speed_x = obj.twist.linear.x
            obj_speed_y = obj.twist.linear.y

            obj_acceleration_x = obj.accel.linear.x
            obj_acceleration_y = obj.accel.linear.y
            obj_angular_velocity = obj.twist.angular.z
            obj_angular_acceleration = obj.accel.angular.z

            # 使用旋转矩阵将目标车辆的位置转换到自车坐标系
            rotation_matrix_ego = np.array([[np.cos(-yaw), -np.sin(-yaw)],
                                            [np.sin(-yaw), np.cos(-yaw)]])

            # 计算目标车辆相对于自车的绝对位置
            dx = absolute_position_x - position_x
            dy = absolute_position_y - position_y

            relative_position = rotation_matrix_ego @ np.array([dx, dy])
            relative_position_x, relative_position_y = relative_position

            # 计算速度差值
            speed_dx = obj_speed_x - ego_speed_x
            speed_dy = obj_speed_y - ego_speed_y

            relative_velocity = rotation_matrix_ego @ np.array([speed_dx, speed_dy])
            relative_speed_x, relative_speed_y = relative_velocity

            # 计算相对加速度
            acceleration_dx = obj_acceleration_x - acceleration_x
            acceleration_dy = obj_acceleration_y - acceleration_y

            relative_acceleration = rotation_matrix_ego @ np.array([acceleration_dx, acceleration_dy])
            relative_acceleration_x , relative_acceleration_y = relative_acceleration

            # 计算相对航向角
            relative_theta = absolute_yaw - yaw
            relative_theta = (relative_theta + np.pi) % (2 * np.pi) - np.pi

            objects.append({
            'absolute_position_x': absolute_position_x,
            'absolute_position_y': absolute_position_y,
            'absolute_yaw': absolute_yaw,
            'absolute_speed_x': obj_speed_x,
            'absolute_speed_y': obj_speed_y,
            'relative_position_x': relative_position_x,
            'relative_position_y': relative_position_y,
            'relative_speed_x': relative_speed_x,
            'relative_speed_y': relative_speed_y,
            'relative_acceleration_x':relative_acceleration_x,
            'relative_acceleration_y':relative_acceleration_y,
            'relative_theta': relative_theta,
            'obj_acceleration_x': obj_acceleration_x,
            'obj_acceleration_y': obj_acceleration_y,
            'obj_angular_velocity': obj_angular_velocity,
            'obj_angular_acceleration': obj_angular_acceleration
            })

            # 打印物体的相关数据
            # print('-------------------- Object --------------------')
            # print(f"Object Position: ({absolute_position_x}, {absolute_position_y})")
            # print(f"Object Yaw: {absolute_yaw}")
            # print(f"Object Speed (in x, y components): ({obj_speed_x}, {obj_speed_y})")
            # print(f"Object Acceleration (in x, y components): ({obj_acceleration_x}, {obj_acceleration_y})")
            # print(f"Object Angular Velocity: {obj_angular_velocity}")
            # print(f"Object Angular Acceleration: {obj_angular_acceleration}")
            # print(f"Relative Position (in x, y components): ({relative_position_x}, {relative_position_y})")
            # print(f"Relative Speed (in x, y components): ({relative_speed_x}, {relative_speed_y})")
            # print(f"Relative Theta: {relative_theta}")
            # print(f"Relative Acceleration (in x, y components): ({relative_acceleration_x}, {relative_acceleration_y})")
            # print('-------------------------------------------------')

        return ego_vehicle, objects
    

    def map_value_to_range(self, value, min_input, max_input, min_output, max_output):
        return (value - min_input) / (max_input - min_input) * (max_output - min_output) + min_output


    def preprocess_data(self, ego_vehicle, objects, target_speed):
        # Extract ego vehicle data
        ego_x = ego_vehicle['position_x']
        ego_y = ego_vehicle['position_y']
        ego_theta = ego_vehicle['yaw']
        ego_speed = ego_vehicle['speed']

        scaled_speed = self.map_value_to_range(ego_speed, 0, 10, -1, 1)
        target_speed = 0
        
        # Create a list to hold all object data
        objects_data = []
        for obj in objects:
            x = obj['relative_position_x']
            y = obj['relative_position_y']
            theta = obj['relative_theta']
            speed_x = obj['relative_speed_x']
            speed_y = obj['relative_speed_y']

            if(-10 < y < 10):
                scaled_relative_position_y = self.map_value_to_range(y, -10, 10, -1, 1)
            elif(-10 > y):
                scaled_relative_position_y = -1
            else:
                scaled_relative_position_y = 1
            
            if(5 < x < 30):
                scaled_relative_position_x = self.map_value_to_range(x, 0, 30, -1, 1)
            elif(5 > x):
                scaled_relative_position_x = -1
            else:
                scaled_relative_position_x = 1

            scaled_relative_theta = self.map_value_to_range(theta, -np.pi, np.pi, -1, 1)
            scaled_relative_speed_x = self.map_value_to_range(speed_x, -15, 5, -1, 1)
            scaled_relative_speed_y = self.map_value_to_range(speed_y, -5, 5, -1, 1)

            objects_data.extend([scaled_relative_position_x, scaled_relative_position_y, scaled_relative_theta, 
                                 scaled_relative_speed_x, scaled_relative_speed_y])
            
            # Print the scaled values
            # print(f"scaled_relative_position_x: {scaled_relative_position_x}")
            # print(f"scaled_relative_position_y: {scaled_relative_position_y}")
            # print(f"relative_theta: {scaled_relative_theta}")
            # print(f"scaled_relative_speed_x: {scaled_relative_speed_x}")
            # print(f"scaled_relative_speed_y: {scaled_relative_speed_y}")

        # Combine all data into a single state array
        # state = np.array([ego_x, ego_y, ego_theta, ego_speed, target_speed] + objects_data)
        state = np.array([ego_speed, target_speed])
        # state = np.array([ego_x, ego_y, ego_theta, ego_speed, target_speed])
        # state = np.array([scaled_speed, target_speed])

        return state

    def compute_next_state(self, ego_vehicle, objects, delta_t=0.1):
        # Update the speed of the ego vehicle
        next_speed = ego_vehicle['speed'] + ego_vehicle['acceleration'] * delta_t

        # Update the position of the ego vehicle
        next_x = ego_vehicle['position_x'] + (ego_vehicle['speed'] + ego_vehicle['acceleration']*delta_t/2) * np.cos(ego_vehicle['yaw']) * delta_t
        next_y = ego_vehicle['position_y'] + (ego_vehicle['speed'] + ego_vehicle['acceleration']*delta_t/2) * np.sin(ego_vehicle['yaw']) * delta_t
        
        next_yaw = ego_vehicle['yaw'] + (ego_vehicle['angular_velocity'] + ego_vehicle['angular_acceleration']*delta_t/2) * delta_t

        # Recreate the ego vehicle state dictionary
        next_ego_vehicle = {
            'position_x': next_x,
            'position_y': next_y,
            'yaw': next_yaw,
            'speed': next_speed,
        }

        # Update the state of each object
        next_objects = []
        for obj in objects:
            next_speed_x = obj['relative_speed_x'] + obj['relative_acceleration_x'] * delta_t
            next_speed_y = obj['relative_speed_y'] + obj['relative_acceleration_y'] * delta_t
            next_x = obj['relative_position_x'] + (obj['relative_speed_x'] + obj['relative_acceleration_x'] * delta_t/2) * delta_t
            next_y = obj['relative_position_y'] + (obj['relative_speed_y'] + obj['relative_acceleration_y'] * delta_t/2) * delta_t
            next_theta = obj['absolute_yaw'] + obj['obj_angular_velocity'] * delta_t
            relative_theta = next_theta - next_yaw
            relative_theta = (relative_theta + np.pi) % (2 * np.pi) - np.pi
            next_objects.append({
                'relative_position_x': next_x,
                'relative_position_y': next_y,
                'relative_theta': relative_theta,
                'relative_speed_x': next_speed_x,
                'relative_speed_y': next_speed_y
            })

        # Generate the new state vector
        next_state = self.preprocess_data(next_ego_vehicle, next_objects, self.target_speed)

        return next_state
    
    def compute_reward(self, ego_vehicle, objects):
        threshold_y = 1
        threshold_x = 15

        distance_reward = 0
        speed_reward = -((ego_vehicle['speed'] - self.target_speed) ** 2) / 25

        min_relative_position_x = float('inf')  # 初始化为正无穷大
        for obj in objects:
            relative_position_x = obj['relative_position_x']
            relative_position_y = obj['relative_position_y']

            # 判断是否符合y方向距离小于阈值且x方向距离小于阈值，并且relative_position_x为正
            if abs(relative_position_y) < threshold_y and relative_position_x > 0 and abs(relative_position_x) < threshold_x:
                # 找到最小的 relative_position_x
                min_relative_position_x = min(min_relative_position_x, relative_position_x)

        # # 3. 如果找到了符合条件的最小 relative_position_x，则计算距离惩罚
        # total_penalty = 0.0
        if min_relative_position_x != float('inf'):
            distance_reward = -((7 / abs(min_relative_position_x)) ** 2)
        
        total_reward = speed_reward# + distance_reward

        return total_reward

    def step(self):
        ego_vehicle, objects = self.update_data()

        if not self.init_position_recored:
            self.init_position_recored = True
            self.ego_posotion_x = ego_vehicle['position_x']
            self.ego_posotion_y = ego_vehicle['position_y']
            self.last_vehicle_reset_time = time.time()

        time_durarion = time.time() - self.last_vehicle_reset_time

        state = self.preprocess_data(ego_vehicle, objects, self.target_speed)
        action = self.agent.select_action(state, self.step_count)
        next_state = self.compute_next_state(ego_vehicle, objects)
        reward = self.compute_reward(ego_vehicle, objects)

        if(self.if_collision):
            print("collision detected!")
            done_bool = True
        else:
            done_bool = False

        if(time_durarion > 5):
            distance = math.sqrt((ego_vehicle['position_x'] - self.ego_posotion_x) ** 2 + (ego_vehicle['position_y'] - self.ego_posotion_y) ** 2)
            if(distance > 2):
                self.is_left = True
            
            if(distance < 1 and self.is_left):
                done_bool = True
        
        if(time_durarion > 120):
            done_bool = True

        self.agent.memory.add(state, action, next_state, reward, done_bool)

        self.visualize_data(ego_vehicle, objects, reward)

        self.step_count += 1

        if self.step_count % 10 == 0:
            self.learn()

        scaled_action = (action + 1) * 0.5 * self.max_action

        if(done_bool):
            self.reset()

        return done_bool, scaled_action

    def learn(self):
        self.agent.train(self.step_count)

    def visualize_data(self, ego_vehicle, objects, reward):
        self.writer.add_scalar('real speed', ego_vehicle['speed'], self.step_count)
        # self.writer.add_scalar('real acceleration', ego_vehicle['acceleration'], self.step_count)
        self.writer.add_scalar('reward', reward, self.step_count)