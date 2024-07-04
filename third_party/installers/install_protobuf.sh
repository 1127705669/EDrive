#!/usr/bin/env bash

# Exit script on any error
set -e

# Set the installation prefix to third_party/build/protobuf
INSTALL_PREFIX=$(pwd)/../build/protobuf

# Create the build directory if it does not exist
mkdir -p $INSTALL_PREFIX

# Define the Protocol Buffers source directory
PROTOBUF_SRC_DIR="../protobuf"  # Adjust this path if necessary

# Navigate to the script directory
cd "$(dirname "${BASH_SOURCE[0]}")"

# Ensure the source directory exists
if [ ! -d "${PROTOBUF_SRC_DIR}" ]; then
    echo "Error: Protocol Buffers source directory not found at ${PROTOBUF_SRC_DIR}"
    exit 1
fi

# Build and install Protocol Buffers
pushd "${PROTOBUF_SRC_DIR}"
mkdir -p cmake/build && cd cmake/build
cmake .. -DCMAKE_INSTALL_PREFIX="${INSTALL_PREFIX}" -DBUILD_SHARED_LIBS=ON -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
make install
popd

# Clean up
# Note: No cleanup necessary as we are using the local source directory

echo "Protocol Buffers has been successfully installed to ${INSTALL_PREFIX}"
