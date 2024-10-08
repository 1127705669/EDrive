import rospy
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from planning.msg import ADCTrajectory  # 确保替换为你的实际包名
from common.msg import TrajectoryPoint  # 确保替换为你的实际包名
from collections import deque
from environment import Environment
from ddpg_agent import DDPG
import numpy as np
import pandas as pd  # 导入 pandas 库

TRAJECTORY_POINTS = 828  # 固定的 TrajectoryPoint 数量
TARGET_SPEED = 20 / 3.6  # 20 km/h in m/s

def flatten_and_reshape(states):
    return np.array(states).flatten().reshape(1, -1)

class ROSNode:
    def __init__(self):
        rospy.init_node('ddpg_controller', anonymous=True)
        rospy.Subscriber('/EDrive/localization/position', Odometry, self.odometry_callback)
        rospy.Subscriber('/EDrive/planning/trajectory', ADCTrajectory, self.trajectory_callback)
        
        self.odometry_data = deque(maxlen=20)
        self.trajectory_data = deque(maxlen=20)
        self.env = Environment(TARGET_SPEED)  # 初始化环境

        self.ddpg_model = DDPG(state_dim=150, action_dim=50, max_action=10)

        self.trajectory_pub = rospy.Publisher('/EDrive/planning/RLtrajectory', ADCTrajectory, queue_size=10)
        self.mpc_weight_pub = rospy.Publisher('/EDrive/planning/MpcWeight', Float64, queue_size=10)
        self.mpc_target_speed_pub = rospy.Publisher('/EDrive/planning/MpcTargetSpeed', Float64, queue_size=10)

    def odometry_callback(self, data):
        self.odometry_data.append(data)

    def trajectory_callback(self, data):
        self.trajectory_data.append(data)

    def find_nearest_index(self, position, trajectory_points):
        distances = [np.linalg.norm(np.array([point.path_point.x, point.path_point.y]) - np.array([position.x, position.y])) for point in trajectory_points]
        return np.argmin(distances)

    def get_points_around(self, index, num_points_before, num_points_after):
        indices = [(index - i) % TRAJECTORY_POINTS for i in range(num_points_before, 0, -1)] + [(index + i) % TRAJECTORY_POINTS for i in range(num_points_after + 1)]
        return indices

    def extract_point_state(self, point):
        state = [point.path_point.x, point.path_point.y, point.v]
        return state

    def print_debug_info(self, indices, states, nearest_index, updated_speeds, rewards):
        # 使用 pandas DataFrame 更好地格式化和打印状态信息
        states_df = pd.DataFrame(states, columns=['x', 'y', 'v'])
        states_df['index'] = indices
        states_df['nearest'] = ['<-- Nearest' if idx == nearest_index else '' for idx in indices]
        states_df['updated_v'] = updated_speeds
        states_df['reward'] = rewards
        print("States to model:\n", states_df.to_string(index=False))

    def find_and_extract_points(self):
        if not self.trajectory_data or not self.odometry_data:
            return None, None, None

        # 使用最新的轨迹数据帧
        latest_trajectory = self.trajectory_data[-1]

        # 使用最新的自车数据
        car_position = self.odometry_data[-1].pose.pose.position

        # 找到距离自车最近的路径点索引
        nearest_index = self.find_nearest_index(car_position, latest_trajectory.trajectory_point)

        # 提取前后指定数量的点
        indices = self.get_points_around(nearest_index, 20, 29)

        # 提取指定点的部分数据
        states = [self.extract_point_state(latest_trajectory.trajectory_point[i]) for i in indices]

        return indices, states, nearest_index

def main():
    ros_node = ROSNode()
    
    rate = rospy.Rate(10)  # 10 Hz，即每100ms运行一次
    while not rospy.is_shutdown():
        if len(ros_node.odometry_data) < 20 or len(ros_node.trajectory_data) < 20:
            rate.sleep()
            continue

        indices, states, nearest_index = ros_node.find_and_extract_points()
        if indices is None or states is None:
            rate.sleep()
            continue

        # 将 states 展平并 reshape 成新的变量
        flattened_states = flatten_and_reshape(states)

        # 使用 DDPG 模型预测这些点的速度
        updated_speeds = ros_node.ddpg_model.predict_speeds(flattened_states)

        for i, idx in enumerate(indices):
            ros_node.trajectory_data[-1].trajectory_point[idx].v = updated_speeds[i]

        # 发布更新后的轨迹
        ros_node.trajectory_pub.publish(ros_node.trajectory_data[-1])

        # 初始化环境中的状态
        initial_states = [[point.path_point.x, point.path_point.y, point.v] for point in ros_node.trajectory_data[-1].trajectory_point]
        ros_node.env.reset(initial_states)

        # 更新环境中的状态
        for i, idx in enumerate(indices):
            current_x = ros_node.trajectory_data[-1].trajectory_point[idx].path_point.x
            current_y = ros_node.trajectory_data[-1].trajectory_point[idx].path_point.y
            current_speed = ros_node.odometry_data[-1].twist.twist.linear.x
            ros_node.env.update_current_state(i, current_x, current_y, current_speed)

        # 环境步进并训练
        next_states, rewards, dones = ros_node.env.step(updated_speeds)

        print(rewards)

        # 调用调试打印函数
        # ros_node.print_debug_info(indices, states, nearest_index, updated_speeds, rewards)

        flattened_next_state = flatten_and_reshape(next_states)

        # 确保经验回放缓冲区中添加的是整组数据
        ros_node.ddpg_model.add_to_replay_buffer((flattened_states, flattened_next_state, updated_speeds, rewards, not any(dones)))
        ros_node.ddpg_model.train(1)  # 每步训练一次

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
