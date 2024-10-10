import rospy
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from collections import deque
from environment import Environment
import numpy as np
import pandas as pd
from torch.utils.tensorboard import SummaryWriter

TARGET_SPEED = 21.6 / 3.6  # 20 km/h in m/s

class ROSNode:
    def __init__(self):
        # 初始化 TensorBoard SummaryWriter
        self.writer = SummaryWriter('runs/ddpg_training')

        rospy.init_node('RL_planner', anonymous=True)

        # 初始化环境，并将 writer 传递给环境和强化学习代理
        self.env = Environment(target_speed=TARGET_SPEED, writer=self.writer)

        self.odometry_queue = deque(maxlen=20)
        self.imu_queue = deque(maxlen=20)
        self.mpc_weight = deque(maxlen=20)
        self.mpc_target_speed = deque(maxlen=20)

        rospy.Subscriber('/EDrive/localization/position', Odometry, self.odometry_callback)
        rospy.Subscriber("/carla/ego_vehicle/imu", Imu, self.imu_callback)

        self.mpc_weight_pub = rospy.Publisher('/EDrive/planning/MpcWeight', Float64, queue_size=10)
        self.mpc_target_speed_pub = rospy.Publisher('/EDrive/planning/MpcTargetSpeed', Float64, queue_size=10)

    def odometry_callback(self, data):
        self.odometry_queue.append(data)

    def imu_callback(self, data):
        self.imu_queue.append(data)

def main():
    
    ros_node = ROSNode()

    # 100 Hz, running every 10 ms
    rate = rospy.Rate(10)

    step_count = 0  # 初始化步计数器

    while not rospy.is_shutdown():
        if len(ros_node.odometry_queue) >= 20 and len(ros_node.imu_queue) >= 20:
            ros_node.env.update_data(ros_node.odometry_queue, ros_node.imu_queue)

            # 执行环境的一步，并获取 target_speed
            next_state, reward, done, target_speed = ros_node.env.step(step_count)

            # 将经验存储到经验回放池中
            ros_node.env.agent.remember(ros_node.env.state, target_speed, reward, next_state, done)

            if done:
                ros_node.env.reset()

            # 定期学习
            step_count += 1
            if step_count % 10 == 0:  # 每10步进行一次学习
                ros_node.env.agent.learn(step_count)

            ros_node.mpc_target_speed.append(target_speed*10)
        
            publish_message(ros_node)
        rate.sleep()

def publish_message(ros_node):
    ros_node.mpc_weight.append(10)
    ros_node.mpc_weight_pub.publish(ros_node.mpc_weight[-1])
    ros_node.mpc_target_speed_pub.publish(ros_node.mpc_target_speed[-1])

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
