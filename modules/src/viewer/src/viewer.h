/******************************************************************************
 * Copyright 2022 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

/**
 * @file viewer.h
 *
 * @brief Declaration of the class Viewer.
 */
#pragma once

#include <string>
#include "common/src/EDrive.h"
#include "common/src/state.h"

#include "viewer/src/common/viewer_agent.h"

#include "viewer/proto/viewer_conf.pb.h"

#include "planning/ADCTrajectory.h"
#include <nav_msgs/Odometry.h>
#include "viewer/VisualizingData.h"
#include "derived_object_msgs/ObjectArray.h"
#include "visualization_msgs/MarkerArray.h"
#include "visualization_msgs/Marker.h"

namespace EDrive {
namespace viewer {

class Viewer : public EDrive::common::EDriveApp {
 public:

  std::string Name() const override;

  EDrive::Result_state Init() override;

  EDrive::Result_state Start() override;

  void Stop() override;

 private:
  // Watch dog timer
  void OnTimer(const ros::TimerEvent &);

  EDrive::Result_state CheckInput();

  void SendData(const ::viewer::VisualizingData *visualizing_data, const visualization_msgs::MarkerArray *viewer_Objects, const visualization_msgs::Marker *viewer_vehicle_data);

  ViewerAgent viewer_agent_;

  ViewerConf viewer_conf_;

  ros::Timer timer_;
  planning::ADCTrajectory trajectory_;
  nav_msgs::Odometry CARLA_location_;
  derived_object_msgs::ObjectArray objects_;

  std::string root_path;
  std::string adapter_conf_file = "/src/viewer/conf/adapter.conf";
  std::string viewer_conf_file = "/src/viewer/conf/viewer.conf";
};

} // namespace viewer
} // namespace EDrive