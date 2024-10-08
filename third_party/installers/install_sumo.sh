#!/usr/bin/env bash

# Exit script on any error
set -e

# Install necessary dependencies for SUMO with GUI
sudo apt-get update
sudo apt-get install -y libx11-dev libxft-dev libxmu-dev libgl1-mesa-dev libglu1-mesa-dev libfox-1.6-dev libgdal-dev

# Set the installation prefix to third_party/build/sumo
INSTALL_PREFIX=$(pwd)/../build/sumo

# Create the build directory if it does not exist
mkdir -p $INSTALL_PREFIX

# Define the SUMO source directory
SUMO_SRC_DIR="../sumo"  # Adjust this path if necessary

# Navigate to the script directory
cd "$(dirname "${BASH_SOURCE[0]}")"

# Ensure the source directory exists
if [ ! -d "${SUMO_SRC_DIR}" ]; then
    echo "Error: SUMO source directory not found at ${SUMO_SRC_DIR}"
    exit 1
fi

# Build and install SUMO
pushd "${SUMO_SRC_DIR}"
mkdir -p build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX="${INSTALL_PREFIX}" -DBUILD_SHARED_LIBS=ON -DCMAKE_BUILD_TYPE=Release -DBUILD_GUI=ON
make -j$(nproc)
make install
popd

echo "SUMO has been successfully installed to ${INSTALL_PREFIX}"
