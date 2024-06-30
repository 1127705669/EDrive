/******************************************************************************
 * Copyright 2023 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#include <ros/ros.h>

#include <fstream>
#include <iostream>
#include <fcntl.h>

#include "google/protobuf/io/zero_copy_stream_impl.h"
#include "google/protobuf/text_format.h"

#include "viewer/src/common/adapters/adapter_manager.h"
#include "common/util/file.h"

namespace EDrive {
namespace viewer {
namespace adapter {

AdapterManager::AdapterManager() {}

void AdapterManager::Observe() {
  for (const auto observe : Instance()->observers_) {
    observe();
  }
}

bool AdapterManager::Initialized() { return Instance()->initialized_; }

void AdapterManager::Init(const std::string &adapter_config_filename) {
  AdapterManagerConfig configs;
  
  EDrive::common::util::GetProtoFromASIIFile(adapter_config_filename, &configs);
  
  AdapterManager::Init(configs);
}

void AdapterManager::Init(const AdapterManagerConfig &configs) {
  if (Initialized()) {
    return;
  }

  Instance()->initialized_ = true;
  if (configs.is_ros()) {
    Instance()->node_handle_.reset(new ros::NodeHandle());
  }

  for (const auto &config : configs.config()) {
    switch (config.type()) {
      case AdapterConfig::CONTROL_COMMAND:
        EnableControlCommand("/carla/ego_vehicle/vehicle_control_cmd", config);
        break;
      case AdapterConfig::PLANNING_TRAJECTORY:
        EnablePlanning("/EDrive/planning", config);
        break;
      case AdapterConfig::VIEWER:
        EnableViewer("/EDrive/viewer", config);
        break;
      case AdapterConfig::VEHICLE_DATA:
        EnableCARLAVehicle("/carla/ego_vehicle/odometry", config);
        break;
      case AdapterConfig::CARLA_OBJECTS:
        EnableCARLAObjects("/carla/ego_vehicle/objects", config);
        break;
      default:
        ROS_INFO("Unknown adapter config type!");
        break;
    }
  }
}

} // namespace adapter
} // namespace viewer
} // namespace EDrive