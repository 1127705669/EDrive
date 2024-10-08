import subprocess
import os
import signal
import sys
import time  # 导入time模块来使用sleep功能

class CarlaLauncher:
    def __init__(self):
        self.launch_process = None

    def start_launch(self, x, y, z, pitch, roll, yaw):
        spawn_point = "{},{},{},{},{},{}".format(x, y, z, pitch, roll, yaw)
        launch_command = [
            "roslaunch",
            "carla_spawn_objects",
            "carla_spawn_objects.launch",
            "spawn_point_ego_vehicle:={}".format(spawn_point)
        ]
        # Ensure any previous launch process is stopped
        self.stop_launch()
        # Use subprocess.Popen to start roslaunch
        self.launch_process = subprocess.Popen(launch_command, preexec_fn=os.setsid)

    def stop_launch(self):
        if self.launch_process:
            # Send SIGTERM to stop roslaunch, applicable for Unix systems
            os.killpg(os.getpgid(self.launch_process.pid), signal.SIGTERM)
            self.launch_process.wait()
            self.launch_process = None
            print("Launch file has been stopped.")

if __name__ == "__main__":
    launcher = CarlaLauncher()
    print("Testing the CarlaLauncher interface...")

    # Test starting the launch
    print("Starting the launch process...")
    launcher.start_launch(-54.6, 20.0, 1.0, 0.0, 0.0, -90.0)
    print("Launch should now be running. Check ROS topics with 'rostopic list'.")

    # Wait for 10 seconds before stopping
    time.sleep(10)
    print("Automatically stopping the launch process after 10 seconds...")
    launcher.stop_launch()

    # Wait for 2 more seconds before restarting
    time.sleep(2)
    print("Restarting the launch process after 2 seconds...")
    launcher.start_launch(-54.6, 20.0, 1.0, 0.0, 0.0, -90.0)
    print("Launch should now be running again. Check ROS topics with 'rostopic list'.")

    # Optional: Wait for user input to finally stop the launch or let it run
    try:
        input("Press Enter to finally stop the launch process...")
    except KeyboardInterrupt:
        pass  # Handle Ctrl+C gracefully

    # Stop the launch
    launcher.stop_launch()
    print("Launch has been stopped finally.")
