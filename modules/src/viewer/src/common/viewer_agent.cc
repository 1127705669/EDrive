/******************************************************************************
 * Copyright 2023 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#include <ros/ros.h>
#include "viewer/src/common/viewer_agent.h"

#include "viewer/src/visual_component/vehicle_state/vehicle_state_visualizer.h"
#include "viewer/src/visual_component/environment/env_handle.h"

namespace EDrive {
namespace viewer {

void ViewerAgent::RegisterControllers(const ViewerConf *viewer_conf) {
  // for (auto active_viewer : viewer_conf->active_viewers()) {
  //   switch (active_viewer) {
  //     case ViewerConf::VEHSTA_VIEWER:
  //       viewer_list_.emplace_back(std::move(new Vehicle_state()));
  //       break;
  //     case ViewerConf::ENV_VIEWER:
  //       viewer_list_.emplace_back(std::move(new Env_handle()));
  //       break;
      
  //     default:
  //       ROS_ERROR("    Unknown active controller type: ");
  //   }
  // }
}

Result_state ViewerAgent::Init(const ViewerConf *viewer_conf) {
  // RegisterControllers(viewer_conf);
  // for(auto &viewer : viewer_list_) {
  //   if (viewer == NULL || EDrive::State_Ok != viewer->Init(viewer_conf)) {
  //     ROS_ERROR("    viewer init failed!");
  //   }
  // }
  return State_Ok;
}

Result_state ViewerAgent::Visualize() {
  // for (auto &viwer : viewer_list_) {
  //   ros::Time start_timestamp = ros::Time::now();
  //   viwer->Visualize(CARLA_location, CARLA_object, visualizing_data, viewer_vehicle_data);
  //   ros::Time end_timestamp = ros::Time::now();
  // }
  return State_Ok;
}

} // namespace viewer
} // namespace EDrive