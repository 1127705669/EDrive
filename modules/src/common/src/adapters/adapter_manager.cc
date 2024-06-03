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
        EnablePlanning("/EDrive/planning/trajectory", config);
        break;
      case AdapterConfig::VIEWER:
        EnableViewer("/EDrive/viewer", config);
        break;
      case AdapterConfig::VEHICLE_DATA:
        EnableVehicle("/carla/ego_vehicle/odometry", config);
        break;
      case AdapterConfig::CARLA_OBJECTS:
        EnableCarlaObjects("/carla/ego_vehicle/objects", config);
        break;
      case AdapterConfig::VIEWER_OBJECTS:
        EnableViewerObjects("/EDrive/viewer/objects", config);
        break;
      case AdapterConfig::PERCEPTION_OBJECTS:
        EnablePerception("/EDrive/perception/objects", config);
        break;
      case AdapterConfig::LOCALIZATION_POSITION:
        EnableLocalization("/EDrive/localization/position", config);
        break;
      case AdapterConfig::VIEWER_PATH:
        EnableViewerPath("/EDrive/viewer/path", config);
        break;
      case AdapterConfig::ROAD_MARKINGS:
        EnableRoadMarkings("/carla/ego_vehicle/semantic_segmentation_front/image", config);
        break;
      case AdapterConfig::ROAD_MARKER:
        EnableRoadMarker("/EDrive/perception/road_lane", config);
        break;
      case AdapterConfig::VEHICLE_POSITION:
        EnableVehicleLocation("/EDrive/localization/position/viewer", config);
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