conda activate carla

cd ../modules
catkin_make -j $(nproc)

cd ../simulator/CARLA_0.9.13/catkin_ws
catkin build