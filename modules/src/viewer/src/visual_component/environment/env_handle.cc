/******************************************************************************
 * Copyright 2023 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#include <ros/ros.h>

#include "viewer/src/visual_component/environment/env_handle.h"

namespace EDrive {
namespace viewer {

Env_handle::Env_handle(derived_object_msgs::ObjectArray *objects, visualization_msgs::MarkerArray *objects_marker_array) {
  ROS_INFO("    registering viewer [environment data]...");
  objects_ = objects;
  objects_marker_array_ = objects_marker_array;
}

EDrive::Result_state Env_handle::Init(const ViewerConf *viewer_conf) {
  return State_Ok;
}

EDrive::Result_state Env_handle::InterfaceMatch() {

  int id = 0; // Unique ID for each marker

  for (const auto& object : objects_->objects){
    visualization_msgs::Marker marker;

    marker.header.frame_id = "map"; // Assuming a global frame called "world"
    marker.header.stamp = ros::Time::now();
    marker.ns = "object_markers";
    marker.id = id++;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;

    // Position
    marker.pose.position.x = object.pose.position.x;
    marker.pose.position.y = object.pose.position.y;
    marker.pose.position.z = object.pose.position.z;

    // Orientation
    marker.pose.orientation.x = object.pose.orientation.x;
    marker.pose.orientation.y = object.pose.orientation.y;
    marker.pose.orientation.z = object.pose.orientation.z;
    marker.pose.orientation.w = object.pose.orientation.w;

    // Scale
    marker.scale.x = object.shape.dimensions[0]; // Example size, adjust as needed
    marker.scale.y = object.shape.dimensions[1];
    marker.scale.z = object.shape.dimensions[2];

    // Color
    marker.color.r = 0.0; // Example color, adjust as needed
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0; // Don't forget to set the alpha!

    marker.lifetime = ros::Duration(0.5); // How long the marker will last before disappearing

    objects_marker_array_->markers.push_back(marker);
  }
  return State_Ok;
}

EDrive::Result_state Env_handle::PublishVisualizationData() {
  return State_Ok;
}

void Env_handle::Stop() {
}

EDrive::Result_state Env_handle::Visualize(){
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