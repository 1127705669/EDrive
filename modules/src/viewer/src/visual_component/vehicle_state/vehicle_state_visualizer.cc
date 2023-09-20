/******************************************************************************
 * Copyright 2023 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#include <ros/ros.h>

#include "viewer/src/visual_component/vehicle_state/vehicle_state_visualizer.h"

namespace EDrive {
namespace viewer {

Vehicle_state::Vehicle_state() {
  ROS_INFO("    Registering viewer [vehicle state]...");
}

EDrive::Result_state Vehicle_state::Init(const ViewerConf *viewer_conf) {
  return State_Ok;
}

EDrive::Result_state Vehicle_state::InterfaceMatch() {
  return State_Ok;
}

EDrive::Result_state Vehicle_state::PublishVisualizationData() {
  return State_Ok;
}

void Vehicle_state::Stop() {
}

EDrive::Result_state Vehicle_state::Visualize(const nav_msgs::Odometry *location, const derived_object_msgs::ObjectArray *object, ::viewer::VisualizingData *visualizing_data){
  Result_state state = State_Failed;
  
  location_ = location;
  visualizing_data_ = visualizing_data;

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