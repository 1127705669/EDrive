#!/usr/bin/env bash

# Exit script on any error
set -e

# Navigate to the third_party/installers directory
cd "$(dirname "${BASH_SOURCE[0]}")/installers"

# Run each install script
echo "Installing tinyxml2..."
bash install_tinyxml2.sh
echo "tinyxml2 installation completed."

echo "Installing gflags..."
bash install_gflags.sh
echo "gflags installation completed."

echo "Installing glog..."
bash install_glog.sh
echo "glog installation completed."

echo "Installing osqp..."
bash install_osqp.sh
echo "osqp installation completed."

echo "Installing osqp-eigen..."
bash install_osqp_eigen.sh
echo "osqp-eigen installation completed."

echo "Installing protocol buffers..."
bash install_protobuf.sh
echo "protocol buffers installation completed."

echo "All third-party libraries have been successfully installed."
