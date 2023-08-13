cd ../modules
catkin_make -j $(nproc)

cd ../simulator/carla_9.12
catkin build