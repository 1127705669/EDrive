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

  trajectory_path_->header.frame_id = "map";

  for (auto point : trajectory_->trajectory_point)
  {
    geometry_msgs::PoseStamped pose_stamped;

    // Set the timestamp and coordinate frame of the point
    pose_stamped.header.stamp = ros::Time::now();
    pose_stamped.header.frame_id = "map";

    // Set point location
    pose_stamped.pose.position.x = point.path_point.x;
    pose_stamped.pose.position.y = point.path_point.y;
    pose_stamped.pose.position.z = point.path_point.z;

    // Set the direction of the point
    pose_stamped.pose.orientation.x = 0.0;
    pose_stamped.pose.orientation.y = 0.0;
    pose_stamped.pose.orientation.z = 0.0;
    pose_stamped.pose.orientation.w = 1.0;

    // Add this point to the path
    trajectory_path_->poses.push_back(pose_stamped);
  }
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