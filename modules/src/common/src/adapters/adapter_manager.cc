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

namespace EDrive {
namespace common {
namespace adapter {

AdapterManager::AdapterManager() {}

bool AdapterManager::Initialized() { return instance()->initialized_; }

void AdapterManager::Init(const std::string &adapter_config_filename) {
  using google::protobuf::TextFormat;
  using google::protobuf::io::FileInputStream;
  using google::protobuf::io::ZeroCopyInputStream;

  AdapterManagerConfig configs;
  int file_descriptor = open(adapter_config_filename.c_str(), O_RDONLY);
  if (file_descriptor < 0) {
    ROS_ERROR("Failed to open file in text mode.");
  }

  ZeroCopyInputStream *input = new FileInputStream(file_descriptor);
  bool success = TextFormat::Parse(input, &configs);
  if (!success) {
    ROS_ERROR("Failed to parse file as text proto.");
  }
  delete input;
  close(file_descriptor);
  
  Init(configs);
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