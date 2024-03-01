/******************************************************************************
 * Copyright 2023 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#include <ros/ros.h>

#include "viewer/src/visual_component/planning/planning_handle.h"

namespace EDrive {
namespace viewer {

Planning_handle::Planning_handle(::planning::ADCTrajectory *trajectory, nav_msgs::Path *trajectory_path) {
  ROS_INFO("    registering viewer [path data]...");
  trajectory_ = trajectory;
  trajectory_path_ = trajectory_path;
}

EDrive::Result_state Planning_handle::Init(const ViewerConf *viewer_conf) {
  return State_Ok;
}

EDrive::Result_state Planning_handle::InterfaceMatch() {
  return State_Ok;
}

EDrive::Result_state Planning_handle::PublishVisualizationData() {
  return State_Ok;
}

void Planning_handle::Stop() {
}

EDrive::Result_state Planning_handle::Visualize(){
  Result_state state = State_Failed;

  state = InterfaceMatch();
  if(State_Ok != state){

  }

  state = PublishVisualizationData();
  if(State_Ok != state){

  }

  return state;
}

} // namespace viewer
} // namespace EDrive