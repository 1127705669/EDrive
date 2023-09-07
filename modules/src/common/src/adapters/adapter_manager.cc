/******************************************************************************
 * Copyright 2023 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#include <ros/ros.h>

#include <fstream>
#include <iostream>
#include <fcntl.h>

#include "google/protobuf/io/zero_copy_stream_impl.h"
#include "google/protobuf/text_format.h"

#include "common/src/adapters/adapter_manager.h"
#include "common/src/util/file.h"

namespace EDrive {
namespace common {
namespace adapter {

AdapterManager::AdapterManager() {}

void AdapterManager::Observe() {
  for (const auto observe : instance()->observers_) {
    observe();
  }
}

bool AdapterManager::Initialized() { return instance()->initialized_; }

void AdapterManager::Init(const std::string &adapter_config_filename) {
  AdapterManagerConfig configs;
  
  EDrive::common::util::GetProtoFromASIIFile(adapter_config_filename, &configs);
  
  AdapterManager::Init(configs);
}

void AdapterManager::Init(const AdapterManagerConfig &configs) {
  if (Initialized()) {
    return;
  }

  instance()->initialized_ = true;
  if (configs.is_ros()) {
    instance()->node_handle_.reset(new ros::NodeHandle());
  }

  for (const auto &config : configs.config()) {
    switch (config.type()) {
      case AdapterConfig::CONTROL_COMMAND:
        EnableControlCommand("/carla/ego_vehicle/vehicle_control_cmd", config);
        break;
      case AdapterConfig::PLANNING_TRAJECTORY:
        EnablePlanning("planning", config);
        break;
      default:
        ROS_INFO("Unknown adapter config type!");
        break;
    }
  }
}

} // adapter
} // common
} // EDrive