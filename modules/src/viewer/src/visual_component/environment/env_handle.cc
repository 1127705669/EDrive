/******************************************************************************
 * Copyright 2023 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#include <ros/ros.h>

#include "viewer/src/visual_component/environment/env_handle.h"

namespace EDrive {
namespace viewer {

Env_handle::Env_handle() {
  ROS_INFO("    Registering viewer [environment data]...");
}

EDrive::Result_state Env_handle::Init(const ViewerConf *viewer_conf) {
  return State_Ok;
}

EDrive::Result_state Env_handle::InterfaceMatch() {
  return State_Ok;
}

EDrive::Result_state Env_handle::PublishVisualizationData() {
  return State_Ok;
}

void Env_handle::Stop() {
}

EDrive::Result_state Env_handle::Visualize(const nav_msgs::Odometry *CARLA_location, const derived_object_msgs::ObjectArray *CARLA_object, 
                                        ::viewer::VisualizingData *visualizing_data, visualization_msgs::Marker *viewer_vehicle_data){
  Result_state state = State_Failed;
  
  location_ = CARLA_location;
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