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
import subprocess
import os
import time
from vehicle_generator import VehicleGenerator

class ROSNode:
    def __init__(self):

        rospy.init_node('RL_planner', anonymous=True)

        self.env = Environment()
        self.vehicle_generator = VehicleGenerator()

        self.last_vehicle_reset_time = 0

        self.process = None
 
        self.reset_flag = True
        self.reset_done = False
        self.object_generated = False

        # input
        self.odometry_queue = deque(maxlen=20)
        self.imu_queue = deque(maxlen=20)
        self.objects_queue = deque(maxlen=20)

        # output
        self.mpc_weight = deque(maxlen=20)
        self.mpc_target_speed = deque(maxlen=20)

        rospy.Subscriber('/carla/ego_vehicle/odometry', Odometry, self.odometry_callback)
        rospy.Subscriber('/carla/ego_vehicle/imu', Imu, self.imu_callback)
        rospy.Subscriber('/carla/ego_vehicle/objects', ObjectArray, self.objects_callback)
        rospy.Subscriber('/carla/ego_vehicle/collision', CarlaCollisionEvent, self.collision_callback)
        
        self.mpc_weight_pub = rospy.Publisher('/EDrive/planning/MpcWeight', Float64, queue_size=10)
        self.mpc_target_speed_pub = rospy.Publisher('/EDrive/planning/MpcTargetSpeed', Float64, queue_size=10)

        rospy.on_shutdown(self.shutdown_handler)

    def reset(self):
        if self.reset_flag:
            self.last_vehicle_reset_time = time.time()
            self.reset_flag = False
            self.odometry_queue_update_flag = False
            self.imu_queue_update_flag = False
            self.objects_queue_update_flag = False
            self.data_ready = False
            self.object_generated = False
            self.vehicle_generator.destroy_vehicle()
            self.terminate_carla_processes()

        time_durarion = time.time() - self.last_vehicle_reset_time

        if(time_durarion > 2 and not self.object_generated):
            self.spawn_ego_vehicle()
            # self.vehicle = self.vehicle_generator.spawn_vehicle(location=(-54.1, -30.0, 1.0), rotation=(0, 90, 0))
            self.object_generated = True
        
        if(time_durarion > 5):
            self.reset_done = True

    def check_data_ready(self):
        if self.odometry_queue_update_flag and self.imu_queue_update_flag and self.objects_queue_update_flag:
            self.data_ready = True

    def check_carla_processes(self):
        try:
            completed_process = subprocess.run(
                ["pgrep", "-f", "roslaunch.*carla_spawn_objects.launch"], 
                text=True, capture_output=True, check=False
            )

            if completed_process.stdout:
                self.process = completed_process.stdout.strip()
            else:
                self.process = None

        except subprocess.CalledProcessError as e:
            rospy.loginfo("未找到任何 carla_spawn_objects roslaunch 进程。")
            self.process = None
        except Exception as e:
            rospy.logerr(f"检查 roslaunch 进程时发生错误: {e}")
            self.process = None
        
    def terminate_carla_processes(self):
        try:
            rospy.loginfo("正在终止 carla_spawn_objects roslaunch 进程...")
            subprocess.run(["pkill", "-f", "roslaunch.*carla_spawn_objects.launch"], check=False)
        except Exception as e:
            rospy.logerr(f"终止 roslaunch 进程时发生错误: {e}")

    def spawn_ego_vehicle(self):
        try:
            command = ["roslaunch", "carla_spawn_objects", "carla_spawn_objects.launch"]
            self.process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

            rospy.loginfo("正在生成新的 ego_vehicle...")

        except Exception as e:
            rospy.logerr(f"生成 ego_vehicle 时发生错误: {e}")

    def odometry_callback(self, data):
        if data is not None:
            self.odometry_queue.append(data)
            self.env.odometry_queue.append(data)
            self.odometry_queue_update_flag = True
            self.check_data_ready()

    def imu_callback(self, data):
        if data is not None:
            self.imu_queue.append(data)
            self.env.imu_queue.append(data)
            self.imu_queue_update_flag = True
            self.check_data_ready()

    def objects_callback(self, data):
        if data is not None:
            self.objects_queue.append(data)
            self.env.objects_queue.append(data)
            self.objects_queue_update_flag = True
            self.check_data_ready()
        
    def collision_callback(self, data):
        self.env.collision_detected()

    def shutdown_handler(self):
        print("closing ros node...")
        self.vehicle_generator.destroy_vehicle()
        self.terminate_carla_processes()

def main():
    
    ros_node = ROSNode()

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():

        if not ros_node.reset_done or ros_node.reset_flag:
            ros_node.reset()
            continue
        
        if(ros_node.data_ready):
            done, target_speed = ros_node.env.step()

            if done:
                ros_node.reset_flag = True
                ros_node.reset_done = False

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
