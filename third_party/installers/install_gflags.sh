#!/usr/bin/env bash

# Exit script on any error
set -e

# Set the installation prefix to third_party/build/gflags
INSTALL_PREFIX=$(pwd)/../build/gflags

# Create the build directory if it does not exist
mkdir -p $INSTALL_PREFIX

# Define the gflags source directory
GFLAGS_SRC_DIR="../gflags"  # Adjust this path if necessary

# Navigate to the script directory
cd "$(dirname "${BASH_SOURCE[0]}")"

# Ensure the source directory exists
if [ ! -d "${GFLAGS_SRC_DIR}" ]; then
    echo "Error: GFLAGS source directory not found at ${GFLAGS_SRC_DIR}"
    exit 1
fi

# Build and install gflags
pushd "${GFLAGS_SRC_DIR}"
mkdir -p build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX="${INSTALL_PREFIX}" -DBUILD_SHARED_LIBS=ON -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
make install
popd

# Clean up
# Note: No cleanup necessary as we are using the local source directory

echo "gflags has been successfully installed to ${INSTALL_PREFIX}"
