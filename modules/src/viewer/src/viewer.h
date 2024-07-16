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

#include "viewer/viewer_agent/viewer_agent_base.h"

#include "viewer/proto/viewer_conf.pb.h"

#include "planning/ADCTrajectory.h"
#include <nav_msgs/Odometry.h>
#include "viewer/VisualizingData.h"
#include "derived_object_msgs/ObjectArray.h"
#include "visualization_msgs/MarkerArray.h"
#include "visualization_msgs/Marker.h"
#include "planning/ADCTrajectory.h"
#include <nav_msgs/Path.h>

namespace EDrive {
namespace viewer {

class Viewer : public EDrive::common::EDriveApp {
 public:

  std::string Name() const override;

  common::Result_state Init() override;

  common::Result_state Start() override;

  void Stop() override;

  void ProcessData();

 private:
  // Watch dog timer
  void OnTimer(const ros::TimerEvent &);

  /**
   * @brief
   * Register new Agents. If you need to add a new type of Agent,
   * You should first register your Agent in this function.
   */
  void RegisterAgents(const ViewerConf *viewer_conf_);

  common::Result_state CheckInput();

  void Publish(visualization_msgs::MarkerArray *objects_marker_array);

  ros::Timer timer_;
  ViewerConf viewer_conf_;

  std::vector<std::unique_ptr<ViewerAgentBase>> agent_list_;

  nav_msgs::Odometry ego_vehicle_odometry_;;
  visualization_msgs::Marker ego_vehicle_marker_;

  derived_object_msgs::ObjectArray objects_;
  visualization_msgs::MarkerArray objects_marker_array_;

  ::planning::ADCTrajectory trajectory_;
  nav_msgs::Path trajectory_path_;

  std::string root_path;
  std::string adapter_conf_file = "/src/viewer/conf/adapter.conf";
  std::string viewer_conf_file = "/src/viewer/conf/viewer.conf";
};

} // namespace viewer
} // namespace EDrive