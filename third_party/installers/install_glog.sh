#!/usr/bin/env bash

# Exit script on any error
set -e

# Set the installation prefix to third_party/build/glog
INSTALL_PREFIX=$(pwd)/../build/glog

# Create the build directory if it does not exist
mkdir -p $INSTALL_PREFIX

# Define the glog source directory
GLOG_SRC_DIR="../glog"  # Adjust this path if necessary

# Navigate to the script directory
cd "$(dirname "${BASH_SOURCE[0]}")"

# Ensure the source directory exists
if [ ! -d "${GLOG_SRC_DIR}" ]; then
    echo "Error: GLOG source directory not found at ${GLOG_SRC_DIR}"
    exit 1
fi

# Build and install glog
pushd "${GLOG_SRC_DIR}"
mkdir -p build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX="${INSTALL_PREFIX}" -DBUILD_SHARED_LIBS=ON -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
make install
popd

# Clean up
# Note: No cleanup necessary as we are using the local source directory

echo "glog has been successfully installed to ${INSTALL_PREFIX}"
