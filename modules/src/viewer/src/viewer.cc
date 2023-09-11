/******************************************************************************
 * Copyright 2022 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#include <ros/ros.h>
#include "viewer/src/viewer.h"

#include "common/src/adapters/adapter_manager.h"
#include "common/src/util/file.h"

#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

namespace EDrive {
namespace viewer {

using EDrive::Result_state;
using EDrive::common::adapter::AdapterManager;

std::string Viewer::Name() const { return "viewer"; }

Result_state Viewer::CheckInput() {
  // snapshot all coming data
  AdapterManager::Observe();

  auto trajectory_adapter = AdapterManager::GetPlanning();
  trajectory_ = trajectory_adapter->GetLatestObserved();
  return State_Ok;
}

EDrive::Result_state Viewer::Init(){
  ROS_INFO("Viewer init, starting...");

  root_path = EDrive::common::util::GetRootPath();
  adapter_conf_file = root_path + adapter_conf_file;
  viewer_conf_file = root_path + viewer_conf_file;

  ROS_INFO("  registering node: %s", Name().c_str());
  AdapterManager::Init(adapter_conf_file);

  return State_Ok;
}

EDrive::Result_state Viewer::Start(){
  timer_ = EDrive::common::adapter::AdapterManager::CreateTimer(ros::Duration(viewer_period), 
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
}

} // namespace viewer
} // namespace EDrive