/******************************************************************************
 * Copyright 2022 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#include <ros/ros.h>
#include "viewer/src/viewer.h"

#include "common/src/adapters/adapter_manager.h"
#include "viewer/src/common/viewer_agent.h"
#include "common/src/util/file.h"

#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

namespace EDrive {
namespace viewer {

using EDrive::Result_state;
using EDrive::common::adapter::AdapterManager;

std::string Viewer::Name() const { return "EDrive_viewer"; }

Result_state Viewer::CheckInput() {
  // snapshot all coming data
  AdapterManager::Observe();

  auto trajectory_adapter = AdapterManager::GetPlanning();
  trajectory_ = trajectory_adapter->GetLatestObserved();

  auto location_adapter = AdapterManager::GetVehicle();
  location_ = location_adapter->GetLatestObserved();
  
  return State_Ok;
}

Result_state Viewer::Init(){
  ROS_INFO("Viewer init, starting...");

  root_path = EDrive::common::util::GetRootPath();
  adapter_conf_file = root_path + adapter_conf_file;
  viewer_conf_file = root_path + viewer_conf_file;

  ROS_INFO("  registering node: %s", Name().c_str());
  AdapterManager::Init(adapter_conf_file);

  ROS_INFO("  viewer init, starting...");
  EDrive::common::util::GetProtoFromASIIFile(viewer_conf_file, &viewer_conf_);
  if(State_Ok != viewer_agent_.Init(&viewer_conf_)) {
    ROS_ERROR("    controller agent init failed, stopping...");
  }

  return State_Ok;
}

Result_state Viewer::Start(){
  // set initial vehicle state by cmd
  // need to sleep, because advertised channel is not ready immediately
  // simple test shows a short delay of 80 ms or so
  ROS_INFO("Viewer resetting vehicle state, sleeping for 1000 ms ...");
  ros::Duration(1.0).sleep();

  timer_ = EDrive::common::adapter::AdapterManager::CreateTimer(ros::Duration(viewer_conf_.viewer_period()), 
                                                              &Viewer::OnTimer,
                                                              this);
  ROS_INFO("Viewer init done!");
  ROS_INFO("Viewer started");
  return State_Ok;
}

void Viewer::Stop() {
  
}

void Viewer::OnTimer(const ros::TimerEvent &) {
  ros::Time begin = ros::Time::now();
  Result_state state = CheckInput();
  if(State_Ok != viewer_agent_.Visualize(&location_)) {
    ROS_INFO("visualize failed, stopping...");
  }
}

} // namespace viewer
} // namespace EDrive