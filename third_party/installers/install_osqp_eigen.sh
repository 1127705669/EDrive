#!/usr/bin/env bash

# Exit script on any error
set -e

# Set the installation prefix to third_party/build/osqp-eigen
INSTALL_PREFIX=$(pwd)/../build/osqp-eigen

# Create the build directory if it does not exist
mkdir -p $INSTALL_PREFIX

# Define the OSQP-Eigen source directory
OSQP_EIGEN_SRC_DIR="../osqp-eigen"  # Adjust this path if necessary

# Define the OSQP installation directory
OSQP_INSTALL_DIR=$(pwd)/../build/osqp

# Navigate to the script directory
cd "$(dirname "${BASH_SOURCE[0]}")"

# Ensure the source directory exists
if [ ! -d "${OSQP_EIGEN_SRC_DIR}" ]; then
    echo "Error: OSQP-Eigen source directory not found at ${OSQP_EIGEN_SRC_DIR}"
    exit 1
fi

# Ensure the OSQP installation directory exists
if [ ! -d "${OSQP_INSTALL_DIR}" ]; then
    echo "Error: OSQP installation directory not found at ${OSQP_INSTALL_DIR}"
    exit 1
fi

# Build and install OSQP-Eigen
pushd "${OSQP_EIGEN_SRC_DIR}"
mkdir -p build && cd build
cmake .. -DCMAKE_PREFIX_PATH="${OSQP_INSTALL_DIR}" -DCMAKE_INSTALL_PREFIX="${INSTALL_PREFIX}" -DBUILD_SHARED_LIBS=ON -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
make install
popd

# Clean up
# Note: No cleanup necessary as we are using the local source directory

echo "OSQP-Eigen has been successfully installed to ${INSTALL_PREFIX}"
