import rospy
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from carla_msgs.msg import CarlaCollisionEvent
from derived_object_msgs.msg import ObjectArray
from sensor_msgs.msg import Imu
from collections import deque
from environment import Environment
import numpy as np
import pandas as pd
from torch.utils.tensorboard import SummaryWriter
import subprocess
import os
import time

TARGET_SPEED = 6.0

class ROSNode:
    def __init__(self):
        # 初始化 TensorBoard SummaryWriter
        self.writer = SummaryWriter('runs/ddpg_training')

        rospy.init_node('RL_planner', anonymous=True)

        self.process = None  # 初始化 process 属性，默认为 None
        self.clean_roslaunch_processes()  # 清理之前的 roslaunch 进程

        # 调用生成ego_vehicle函数
        self.spawn_ego_vehicle()

        self.max_action = 12

        # 初始化环境，并将 writer 传递给环境和强化学习代理
        self.env = Environment(self.max_action, target_speed=TARGET_SPEED, writer=self.writer)

        # input
        self.odometry_queue = deque(maxlen=20)
        self.imu_queue = deque(maxlen=20)
        self.objects_queue = deque(maxlen=20)
        self.collision_queue = deque(maxlen=20)

        # output
        self.mpc_weight = deque(maxlen=20)
        self.mpc_target_speed = deque(maxlen=20)

        rospy.Subscriber('/EDrive/localization/position', Odometry, self.odometry_callback)
        rospy.Subscriber("/carla/ego_vehicle/imu", Imu, self.imu_callback)
        rospy.Subscriber('/carla/ego_vehicle/objects', ObjectArray, self.objects_callback)
        rospy.Subscriber('/carla/ego_vehicle/collision', CarlaCollisionEvent, self.collision_callback)
        
        self.mpc_weight_pub = rospy.Publisher('/EDrive/planning/MpcWeight', Float64, queue_size=10)
        self.mpc_target_speed_pub = rospy.Publisher('/EDrive/planning/MpcTargetSpeed', Float64, queue_size=10)
    
    def odometry_callback(self, data):
        self.odometry_queue.append(data)

    def imu_callback(self, data):
        self.imu_queue.append(data)

    def objects_callback(self, data):
        self.objects_queue.append(data)

    def collision_callback(self, data):
        self.collision_queue.append(data)

    def clean_roslaunch_processes(self):
        """
        清理之前可能残留的 carla_spawn_objects roslaunch 进程。
        """
        try:
            # 使用 pgrep 和 pkill 来终止特定的 roslaunch 进程
            rospy.loginfo("正在清理残留的 carla_spawn_objects roslaunch 进程...")
            # 查找并终止包含 carla_spawn_objects.launch 的进程
            os.system("pkill -f 'roslaunch.*carla_spawn_objects.launch'")
        except Exception as e:
            rospy.logerr(f"清理 roslaunch 进程时发生错误: {e}")

    def spawn_ego_vehicle(self):
        """
        调用 roslaunch 启动 carla_spawn_objects.launch 来生成 ego_vehicle。
        """
        # 在生成新的 ego_vehicle 之前，先检查并停止已有的进程
        if self.process:
            rospy.loginfo("已有 ego_vehicle 正在运行，正在停止...")
            self.stop_ego_vehicle()

        try:
            command = ["roslaunch", "carla_spawn_objects", "carla_spawn_objects.launch"]
            # 使用 subprocess.Popen 启动命令，非阻塞
            self.process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

            rospy.loginfo("正在生成新的 ego_vehicle...")

        except Exception as e:
            rospy.logerr(f"生成 ego_vehicle 时发生错误: {e}")

    def stop_ego_vehicle(self):
        """
        停止生成的 ego_vehicle，即停止 roslaunch 进程。
        """
        # 尝试停止由当前对象启动的 roslaunch 进程
        if self.process:
            try:
                rospy.loginfo("正在停止 ego_vehicle...")
                self.process.terminate()  # 或者使用 self.process.kill()
                self.process.wait()  # 等待进程终止，确保完全退出
                rospy.loginfo("ego_vehicle 已停止。")
                self.process = None  # 重置 self.process
            except Exception as e:
                rospy.logerr(f"停止 ego_vehicle 时发生错误: {e}")
        else:
            rospy.logwarn("没有找到生成 ego_vehicle 的进程，由本实例启动的进程可能已退出。")

        # 调用 clean_roslaunch_processes 以确保清理所有相关的残留进程
        self.clean_roslaunch_processes()


def main():
    
    ros_node = ROSNode()

    # 100 Hz, running every 10 ms
    rate = rospy.Rate(10)

    step_count = 0  # 初始化步计数器

    while not rospy.is_shutdown():
        # 检查是否存在 carla_spawn_objects 的 roslaunch 进程
        try:
            result = subprocess.run(["pgrep", "-f", "roslaunch.*carla_spawn_objects.launch"], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            roslaunch_running = (result.returncode == 0)
        except Exception as e:
            rospy.logerr(f"检查 carla_spawn_objects 进程时发生错误: {e}")
            roslaunch_running = False

        if not roslaunch_running:
            continue

        # 确保有足够的里程计和 IMU 数据
        if len(ros_node.odometry_queue) >= 20 and len(ros_node.imu_queue) >= 20:
            ros_node.env.update_data(ros_node.odometry_queue, ros_node.imu_queue, ros_node.objects_queue, ros_node.collision_queue)

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

            ros_node.mpc_target_speed.append(target_speed)
        
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
