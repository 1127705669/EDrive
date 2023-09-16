/******************************************************************************
 * Copyright 2023 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#include <ros/ros.h>
#include "viewer/src/common/viewer_agent.h"

#include "viewer/src/visual_component/vehicle_state/vehicle_state_visualizer.h"

namespace EDrive {
namespace viewer {

void ViewerAgent::RegisterControllers(const ViewerConf *viewer_conf_) {
  for (auto active_viewer : viewer_conf_->active_viewers()) {
    switch (active_viewer) {
      case ViewerConf::VEHSTA_VIEWER:
        viewer_list_.emplace_back(std::move(new Vehicle_state()));
        break;
      default:
        ROS_ERROR("    Unknown active controller type: ");
    }
  }
}

Result_state ViewerAgent::Init(const ViewerConf *viewer_conf_) {
  RegisterControllers(viewer_conf_);
  for(auto &viewer : viewer_list_) {
    if (viewer == NULL || EDrive::State_Ok != viewer->Init(viewer_conf_)) {
      ROS_ERROR("    viewer init failed!");
    }
  }
  return State_Ok;
}

} // namespace viewer
} // namespace EDrive