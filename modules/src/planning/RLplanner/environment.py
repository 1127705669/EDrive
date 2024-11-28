import numpy as np
import torch
# from sac_agent import SACAgent
from td3_agent import TD3Agent
from collections import deque
from tf.transformations import euler_from_quaternion
from torch.utils.tensorboard import SummaryWriter
import math
import time
from low_pass_filter import LowPassFilter
import random
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class Environment:
    def __init__(self, training_mode=True):
        self.training_mode = training_mode
        self.step_count = 0
        self.writer = SummaryWriter('runs/td3')
        self.state_dim = 3
        self.action_dim = 1
        self.max_action = 10
        self.init_speed = 0
        self.agent = TD3Agent(self.state_dim, self.action_dim, self.writer, self.training_mode)
        self.previous_scaled_speed = 0

        self.target_speed = 5
        self.distance_factor = 0

        self.odometry_queue = deque(maxlen=20)
        self.imu_queue = deque(maxlen=20)
        self.objects_queue = deque(maxlen=20)

        self.init_position_recored = False
        self.ego_posotion_x = 0
        self.ego_posotion_y = 0

        self.is_left = False

        self.speed_lilter = LowPassFilter(alpha=0.2)

        self.min_relative_position_x = 0
        self.step_counts_array = []
        self.speeds_array = []
        self.accelerations_array = []
        self.relative_position_x_array = []
        self.lead_vehicle_speeds_array = []

        self.speeds_array_lilter = LowPassFilter(alpha=0.2)
        self.accelerations_array_lilter = LowPassFilter(alpha=0.2)
        self.relative_position_x_array_lilter = LowPassFilter(alpha=0.2)
        self.lead_vehicle_speeds_array_lilter = LowPassFilter(alpha=0.2)

        self.min_speed_x_to_ego = 10

        self.reset()

    def smooth_data(self, data, window_size):
        """ 使用滑动平均平滑数据 """
        window = np.ones(int(window_size))/float(window_size)
        return np.convolve(data, window, 'same')
    
    def plot_speed_and_acceleration(self):
        
        plt.figure(figsize=(18, 6))  # 调整大小以适应三个图

        # 第一个图形：速度
        plt.subplot(1, 3, 2)  # 修改为 1 行 3 列的第 1 个
        plt.plot(self.step_counts_array, self.speeds_array, label='Ego Vehicle Speed', color='blue')
        # if(self.min_speed_x_to_ego < 7):
        plt.plot(self.step_counts_array, self.lead_vehicle_speeds_array, label='Lead Vehicle Speed', color='red')
        plt.xlabel('Time [s]')
        plt.ylabel('Speed [m/s]')
        plt.grid(True)
        plt.legend()

        # 第二个图形：加速度
        plt.subplot(1, 3, 3)  # 修改为 1 行 3 列的第 2 个
        plt.plot(self.step_counts_array, self.accelerations_array, label='Ego Vehicle Acceleration', color='blue')
        plt.xlabel('Time [s]')
        plt.ylabel('Acceleration [$\mathrm{m/s}^2$]')
        plt.grid(True)
        plt.legend()

        # 第三个图形：相对位置
        plt.subplot(1, 3, 1)  # 修改为 1 行 3 列的第 3 个
        plt.plot(self.step_counts_array, self.relative_position_x_array, label='Relative Position', color='blue')
        plt.xlabel('Time [s]')
        plt.ylabel('Relative Position [m]')
        plt.grid(True)
        plt.legend()

        plt.tight_layout()
        plt.show()

    def collision_detected(self):
        self.if_collision = True
        self.last_coliision = self.if_collision

    def reset(self):
        self.if_collision = False
        # self.last_coliision = self.if_collision
        self.init_position_recored = False
        self.is_left = False
        self.init_speed = self.map_value_to_range(random.uniform(3, 7), 0, 10, -1, 1)


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

            obj_speed_to_ego = rotation_matrix_ego @ np.array([obj_speed_x, obj_speed_y])
            speed_x_to_ego, speed_y_to_ego = obj_speed_to_ego
            filtered_speed_x_to_ego = self.speed_lilter.update(speed_x_to_ego)

            # 计算相对加速度
            acceleration_dx = obj_acceleration_x - acceleration_x
            acceleration_dy = obj_acceleration_y - acceleration_y

            acceleration_to_ego = rotation_matrix_ego @ np.array([obj_acceleration_x, obj_acceleration_y])
            acceleration_x_to_ego , acceleration_y_to_ego = acceleration_to_ego

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
            'obj_angular_acceleration': obj_angular_acceleration,
            'speed_x_to_ego': speed_x_to_ego,
            'speed_y_to_ego': speed_y_to_ego,
            'acceleration_x_to_ego': acceleration_x_to_ego,
            'acceleration_y_to_ego': acceleration_y_to_ego
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


    def preprocess_data(self, ego_vehicle, objects, target_speed, is_current):
        # Extract ego vehicle data
        ego_speed = ego_vehicle['speed']
        scaled_speed = self.map_value_to_range(ego_speed, 0, 10, 0, 1)
        self.previous_scaled_speed = scaled_speed
        scaled_target_speed = 0
        self.target_speed = 5
        self.min_speed_x_to_ego = 10
        
        # Create a list to hold all object data
        min_scaled_relative_position_x = 1.0
        self.min_relative_position_x = 105

        for obj in objects:
            x = obj['relative_position_x']
            y = obj['relative_position_y']
            theta = obj['relative_theta']
            speed_x_to_ego = obj['speed_x_to_ego']

            scaled_relative_theta = self.map_value_to_range(theta, -np.pi, np.pi, -1, 1)

            if abs(y) < 2 and 5 < x < 105 and abs(theta) < np.pi/2:
                

                # 找到最小的 relative_position_x
                self.min_relative_position_x = min(self.min_relative_position_x, x)
                self.min_relative_position_x = max(self.min_relative_position_x,5)
                scaled_relative_position_x = self.map_value_to_range(self.min_relative_position_x, 5, 105, 0, 1)
                if(self.min_relative_position_x < 5):
                    scaled_relative_position_x = -1
                elif(self.min_relative_position_x > 105):
                    scaled_relative_position_x = 1
                self.distance_factor = 1 / (1 + np.exp(0.8 * (self.min_relative_position_x - 11.0)))
                
                self.min_speed_x_to_ego = min(self.min_speed_x_to_ego, speed_x_to_ego)
                
                # self.distance_factor = ((20 - min_relative_position_x) / 15)
                # if(self.distance_factor > 1):
                #     self.distance_factor = 1
                # if(self.distance_factor < 0):
                #     self.distance_factor = 0
                min_scaled_relative_position_x = scaled_relative_position_x
                
                # scaled_target_speed = self.distance_factor * (scaled_target_speed - scaled_speed) + scaled_speed

        self.target_speed = (1 - self.distance_factor) * 5 + self.distance_factor * self.min_speed_x_to_ego
        scaled_target_speed = self.map_value_to_range(self.target_speed, 0, 10, 0, 1)

        if(is_current):
            print(self.min_relative_position_x)

        # Combine all data into a single state array
        state = np.array([scaled_speed] + [scaled_target_speed] + [min_scaled_relative_position_x])

        return state
    
    def compute_reward(self, ego_vehicle, objects, state):
        done_bool = False

        stop_reward = 0
        speed_diff_reward = 0
        speed_reward = 0
        distance_reward = 0

        # for obj in objects:
        #     x = obj['relative_position_x']
        #     y = obj['relative_position_y']
        #     theta = obj['relative_theta']

        #     # 判断是否符合y方向距离小于阈值且x方向距离小于阈值，并且relative_position_x为正
        #     if abs(y) < 2 and 5 < x < 105 and abs(theta) < np.pi/2:
        #         min_relative_position_x = min(min_relative_position_x, x)


        # previous_speed = self.map_value_to_range(self.previous_scaled_speed, -1, 1, 0, 10)
        
        # difusion_reward = - (abs(previous_speed - ego_vehicle['speed']))**2

        # target_speed = self.map_value_to_range(state[1], -1, 1, 0, 10)
        # init_speed = self.map_value_to_range(self.init_speed, -1, 1, 0, 10)

        self.writer.add_scalar('self.distance_factor', self.distance_factor, self.step_count)
        self.writer.add_scalar('self.min_relative_position_x', self.min_relative_position_x, self.step_count)

        # init_speed_reward = (1 - self.distance_factor) * (-(abs(ego_vehicle['speed'] - init_speed)**2))
        # 追踪前车车速的奖励
        target_speed_reward = (-(abs(ego_vehicle['speed'] - self.target_speed)**2)) / 25

        final_reward = target_speed_reward
        # if(abs(ego_vehicle['speed'] - self.target_speed) < 0.3):
        #     final_reward += 0.5
        # self.writer.add_scalars('Speeds rewards', {'init_speed_reward': init_speed_reward, 'target_speed_reward': target_speed_reward}, self.step_count)
        self.writer.add_scalar('speed_reward', final_reward, self.step_count)

        # speed_reward = - ((abs(target_speed - ego_vehicle['speed']))**2)/25
        # if(abs(target_speed - ego_vehicle['speed']) < 0.3):
        #     speed_reward += 1

        
        
        # if(min_relative_position_x != float('inf')):
        distance_reward = np.exp(-((self.min_relative_position_x - 8) ** 2) / (2 * 2 ** 2)) * (1 - (abs(ego_vehicle['speed'] - self.target_speed)) / 5)
        if(abs(self.min_relative_position_x -7.5) < 0.5):
            distance_reward += (1 - (abs(ego_vehicle['speed'] - self.target_speed)) / 5)
        self.writer.add_scalar('distance_reward', distance_reward, self.step_count)
        if(self.min_relative_position_x < 6):
            done_bool = True
            stop_reward = -1

        self.writer.add_scalar('self.min_relative_position_x', self.min_relative_position_x, self.step_count)
        # if(5.5 < min_relative_position_x < 7.5):
        #     stop_reward = -1

        # if(7.5 < min_relative_position_x < 9.5 and ego_vehicle['speed'] < 1):
        #     if(min_relative_position_x > 8.5):
        #         stop_reward = 9.5 - min_relative_position_x
        #     else:
        #         stop_reward = 1 #- ((min_relative_position_x - 7.5) ** 2) + 1

        # if(min_relative_position_x != float('inf') and relative_speed_min != float('inf')):
        # print(f"Distance Factor: {self.distance_factor:.2f}, Relative Speed Min: {relative_speed_min:.2f}, Speed Diff Reward: {speed_diff_reward:.2f}, Stop Reward: {stop_reward:.2f}")

        # if(relative_speed_min > -0.2 and 6.5 < min_relative_position_x < 8.5):
        #     stop_reward = - ((min_relative_position_x - 7.5) ** 2) + 1
        #     print(stop_reward)
        
        # if(min_relative_position_x < 6.5):
        #     stop_reward = -1
        #     print(22222222222222222222222222222222)

        # if min_relative_position_x == float('inf'):
        #     speed_reward = -abs(ego_vehicle['speed'] - target_speed)
        
        total_reward = final_reward + distance_reward + stop_reward# + difusion_reward

        return total_reward, done_bool

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
            next_speed_x = obj['speed_x_to_ego'] + obj['acceleration_x_to_ego'] * delta_t
            next_speed_y = obj['speed_y_to_ego'] + obj['acceleration_y_to_ego'] * delta_t
            next_x = obj['relative_position_x'] + (obj['relative_speed_x'] + obj['relative_acceleration_x'] * delta_t/2) * delta_t
            next_y = obj['relative_position_y'] + (obj['relative_speed_y'] + obj['relative_acceleration_y'] * delta_t/2) * delta_t
            next_theta = obj['absolute_yaw'] + obj['obj_angular_velocity'] * delta_t
            relative_theta = next_theta - next_yaw
            relative_theta = (relative_theta + np.pi) % (2 * np.pi) - np.pi
            next_objects.append({
                'relative_position_x': next_x,
                'relative_position_y': next_y,
                'relative_theta': relative_theta,
                'speed_x_to_ego': next_speed_x,
                'speed_y_to_ego': next_speed_y
            })

        # Generate the new state vector
        next_state = self.preprocess_data(next_ego_vehicle, next_objects, self.target_speed, is_current=False)

        return next_state

    def step(self):
        ego_vehicle, objects = self.update_data()

        if not self.init_position_recored:
            self.init_position_recored = True
            self.ego_posotion_x = ego_vehicle['position_x']
            self.ego_posotion_y = ego_vehicle['position_y']
            self.last_vehicle_reset_time = time.time()

        time_durarion = time.time() - self.last_vehicle_reset_time

        state = self.preprocess_data(ego_vehicle, objects, self.target_speed, is_current=True)

        self.step_counts_array.append(self.step_count * 0.1)
        self.speeds_array.append(self.speeds_array_lilter.update(ego_vehicle['speed']))
        self.accelerations_array.append(self.accelerations_array_lilter.update(ego_vehicle['acceleration']))
        self.relative_position_x_array.append(self.relative_position_x_array_lilter.update(self.min_relative_position_x - 5))
        if(self.min_speed_x_to_ego > 8):
            self.min_speed_x_to_ego = 0
        self.lead_vehicle_speeds_array.append(self.lead_vehicle_speeds_array_lilter.update(self.min_speed_x_to_ego))

        action = self.agent.select_action(state)
        reward, done_bool = self.compute_reward(ego_vehicle, objects, state)
        next_state = self.compute_next_state(ego_vehicle, objects)

        self.writer.add_scalars('Speeds', {'real_speed': state[0], 'target_speed': state[1]}, self.step_count)
        self.writer.add_scalar('scaled relative position_x', state[2], self.step_count)

        # if(self.if_collision and self.is_left):
        #     print("collision detected!")
        #     done_bool = True

        if(time_durarion > 5):
            distance = math.sqrt((ego_vehicle['position_x'] - self.ego_posotion_x) ** 2 + (ego_vehicle['position_y'] - self.ego_posotion_y) ** 2)
            if(distance > 2):
                self.is_left = True
            
            if(ego_vehicle['position_y'] < -100 and self.is_left):
                done_bool = True
        
        if(time_durarion > 120):
            done_bool = True

        self.agent.memory.add(state, action, next_state, reward, done_bool)

        self.visualize_data(ego_vehicle, objects, reward)

        self.step_count += 1

        if self.step_count % 10 == 0 and self.training_mode:
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