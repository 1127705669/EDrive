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

TARGET_SPEED = 35 / 3.6  # 20 km/h in m/s

class ROSNode:
    def __init__(self):
        rospy.init_node('ddpg_controller', anonymous=True)
        rospy.Subscriber('/EDrive/localization/position', Odometry, self.odometry_callback)
        rospy.Subscriber('/EDrive/planning/trajectory', ADCTrajectory, self.trajectory_callback)
        
        self.odometry_data = deque(maxlen=20)
        self.trajectory_data = deque(maxlen=20)
        self.mpc_weight = deque(maxlen=20)
        self.mpc_target_speed = deque(maxlen=20)

        # self.env = Environment(TARGET_SPEED)  # 初始化环境

        # self.ddpg_model = DDPG(state_dim=150, action_dim=50, max_action=10)

        self.trajectory_pub = rospy.Publisher('/EDrive/planning/RLtrajectory', ADCTrajectory, queue_size=10)
        self.mpc_weight_pub = rospy.Publisher('/EDrive/planning/MpcWeight', Float64, queue_size=10)
        self.mpc_target_speed_pub = rospy.Publisher('/EDrive/planning/MpcTargetSpeed', Float64, queue_size=10)

    def odometry_callback(self, data):
        self.odometry_data.append(data)

    def trajectory_callback(self, data):
        self.trajectory_data.append(data)

def publish_message(ros_node):
    ros_node.mpc_weight_pub.publish(ros_node.mpc_weight[-1])
    ros_node.mpc_target_speed_pub.publish(ros_node.mpc_target_speed[-1])
    
def main():
    ros_node = ROSNode()
    
    rate = rospy.Rate(10)  # 10 Hz, running every 100 ms
    while not rospy.is_shutdown():
        # Ensure there's enough odometry and trajectory data
        if len(ros_node.odometry_data) < 20 or len(ros_node.trajectory_data) < 20:
            ros_node.mpc_weight.append(1.0)  # Append to deque
            ros_node.mpc_target_speed.append(1.0)  # Append to deque
            rate.sleep()
            continue

        # Publish the latest weight and target speed
        publish_message(ros_node)

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass