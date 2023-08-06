/******************************************************************************
 * Copyright 2023 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#include <ros/ros.h>
#include "common/src/adapters/adapter_manager.h"

namespace EDrive {
namespace common {
namespace adapter {

AdapterManager::AdapterManager() {}

void AdapterManager::Init(const std::string &adapter_config_filename) {
  // Parse config file
  AdapterManagerConfig configs;
  configs.add_config();
  Init(configs);
}

void AdapterManager::Init(const AdapterManagerConfig &configs) {
  ROS_INFO("adapter manager init");

  for (const auto &config : configs.config()) {
    switch (config.type()) {
      case AdapterConfig::CONTROL_COMMAND:
        EnableControlCommand("CONTROL", config);
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