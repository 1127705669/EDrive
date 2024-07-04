#!/bin/bash

# Initialize conda to ensure 'conda activate' can be used
eval "$(conda shell.bash hook)"

# Ensure sudo command is available
if ! command -v sudo &> /dev/null
then
    echo "sudo could not be found. Please ensure sudo is installed and configured."
    exit
fi

# Install system dependencies
sudo apt-get update
sudo apt-get install -y ros-noetic-derived-object-msgs ros-noetic-ackermann-msgs libffi7 libomp5

# Check if the 'carla' conda environment already exists
if conda env list | grep -q "carla"; then
    echo "Activating existing 'carla' environment..."
    conda activate carla
else
    echo "Creating and activating 'carla' environment..."
    conda create -n carla python=3.7 -y
    conda activate carla
fi

# Save the current directory
current_dir=$(pwd)

# Dynamically detect Anaconda installation path
conda_path=$(conda info --base)

# Fix the libffi library issue
cd "$conda_path/envs/carla/lib"
if [ -f "libffi.so.7" ]; then
    mv libffi.so.7 libffi.so.7.bak
fi
sudo ln -s /lib/x86_64-linux-gnu/libffi.so.7.1.0 libffi.so.7
sudo ldconfig

# Return to the saved directory
cd "$current_dir"

# Install Python dependencies
pip install -r requirements.txt
pip install -r catkin_ws/src/ros-bridge/requirements.txt
