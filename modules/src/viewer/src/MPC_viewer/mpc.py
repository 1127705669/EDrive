#!/usr/bin/env python3
import rospy
import matplotlib.pyplot as plt
import matplotlib.dates as mdates
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from carla_msgs.msg import CarlaEgoVehicleControl
import datetime

# Dictionaries to store data and their respective timestamps
data_store = {
    'target_speed': [],
    'target_speed_time': [],
    'throttle': [],
    'throttle_time': [],
    'current_speed': [],
    'current_speed_time': []
}

def speed_callback(data):
    """ Callback for target speed data. """
    data_store['target_speed'].append(data.data)
    data_store['target_speed_time'].append(datetime.datetime.now())

def control_callback(data):
    """ Callback for vehicle control command data. """
    data_store['throttle'].append(data.throttle)
    data_store['throttle_time'].append(datetime.datetime.now())

def odometry_callback(data):
    """ Callback for vehicle odometry data. """
    data_store['current_speed'].append(data.twist.twist.linear.x)
    data_store['current_speed_time'].append(datetime.datetime.now())

def plot_data():
    """ Plots the collected data on the same plot for comparison. """
    plt.figure(figsize=(15, 6))
    formatter = mdates.DateFormatter('%H:%M:%S')
    locator = mdates.MinuteLocator()

    # Convert timestamps to matplotlib dates
    target_dates = mdates.date2num(data_store['target_speed_time'])
    throttle_dates = mdates.date2num(data_store['throttle_time'])
    current_dates = mdates.date2num(data_store['current_speed_time'])

    plt.plot_date(target_dates, data_store['target_speed'], 'r-', label='Target Speed')
    plt.plot_date(throttle_dates, data_store['throttle'], 'g-', label='Throttle Command')
    plt.plot_date(current_dates, data_store['current_speed'], 'b-', label='Current Speed')

    plt.xlabel('Time')
    plt.ylabel('Values')
    plt.title('MPC Tracking Data Comparison')
    plt.legend()
    plt.gca().xaxis.set_major_formatter(formatter)
    plt.gca().xaxis.set_major_locator(locator)
    plt.gcf().autofmt_xdate()  # Auto-format date labels
    plt.tight_layout()
    plt.show()

def listener():
    """ Sets up ROS node and subscribes to topics. """
    rospy.init_node('mpc_tracking', anonymous=True)
    rospy.Subscriber("/EDrive/planning/MpcTargetSpeed", Float64, speed_callback)
    rospy.Subscriber("/carla/ego_vehicle/vehicle_control_cmd", CarlaEgoVehicleControl, control_callback)
    rospy.Subscriber("/EDrive/localization/position", Odometry, odometry_callback)
    
    print("Collecting data...")
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
    finally:
        plot_data()
