/******************************************************************************
 * Copyright 2023 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#include <ros/ros.h>

#include "viewer/src/visual_component/vehicle_state/vehicle_state_visualizer.h"

namespace EDrive {
namespace viewer {

using EDrive::common::Result_state;

Vehicle_state::Vehicle_state() {
  ROS_INFO("    registering viewer [vehicle state]...");
}

Result_state Vehicle_state::Init(const ViewerConf *viewer_conf) {
  return Result_state::State_Ok;
}

Result_state Vehicle_state::InterfaceMatch() {
  return Result_state::State_Ok;
}

Result_state Vehicle_state::PublishVisualizationData() {
  return Result_state::State_Ok;
}

void Vehicle_state::Stop() {
}

Result_state Vehicle_state::Visualize(){
  Result_state state = Result_state::State_Failed;
  
  state = InterfaceMatch();
  if(Result_state::State_Ok != state){

  }

  state = PublishVisualizationData();
  if(Result_state::State_Ok != state){

  }

  return state;
}

} // namespace viewer
} // namespace EDrive