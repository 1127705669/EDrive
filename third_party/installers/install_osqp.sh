#!/usr/bin/env bash

# Exit script on any error
set -e

# Set the installation prefix to third_party/build/osqp
INSTALL_PREFIX=$(pwd)/../build/osqp

# Create the build directory if it does not exist
mkdir -p $INSTALL_PREFIX

# Define the OSQP source directory
OSQP_SRC_DIR="../osqp"  # Adjust this path if necessary

# Navigate to the script directory
cd "$(dirname "${BASH_SOURCE[0]}")"

# Ensure the source directory exists
if [ ! -d "${OSQP_SRC_DIR}" ]; then
    echo "Error: OSQP source directory not found at ${OSQP_SRC_DIR}"
    exit 1
fi

# Build and install OSQP
pushd "${OSQP_SRC_DIR}"
mkdir -p build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX="${INSTALL_PREFIX}" -DBUILD_SHARED_LIBS=ON -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
make install
popd

# Clean up
# Note: No cleanup necessary as we are using the local source directory

echo "OSQP has been successfully installed to ${INSTALL_PREFIX}"
