/******************************************************************************
 * Copyright 2022 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#include <ros/ros.h>
#include "viewer/src/viewer.h"

#include "common/adapters/adapter_manager.h"
#include "common/util/file.h"

#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

#include "viewer/viewer_agent/perception_agent.h"
#include "viewer/viewer_agent/localization_agent.h"

namespace EDrive {
namespace viewer {

using EDrive::common::Result_state;
using EDrive::common::adapter::AdapterManager;

std::string Viewer::Name() const { return "EDrive_viewer"; }

Result_state Viewer::CheckInput() {
  // snapshot all coming data
  AdapterManager::Observe();

  auto trajectory_adapter = AdapterManager::GetPlanning();
  trajectory_ = trajectory_adapter->GetLatestObserved();

  auto ego_vehicle_odometry_adapter = AdapterManager::GetLocalization();
  ego_vehicle_odometry_ = ego_vehicle_odometry_adapter->GetLatestObserved();

  auto objects_adapter = AdapterManager::GetPerception();
  objects_ = objects_adapter->GetLatestObserved();
  
  return Result_state::State_Ok;
}

Result_state Viewer::Init(){
  ROS_INFO("Viewer init, starting...");

  root_path = EDrive::common::util::GetRootPath();
  adapter_conf_file = root_path + adapter_conf_file;
  viewer_conf_file = root_path + viewer_conf_file;

  ROS_INFO("  registering node: %s", Name().c_str());
  AdapterManager::Init(adapter_conf_file);

  ROS_INFO("  viewer init, starting...");
  EDrive::common::util::GetProtoFromASCIIFile(viewer_conf_file, &viewer_conf_);

  RegisterAgents(&viewer_conf_);

  for(auto &agent : agent_list_) {
    agent->Init(&viewer_conf_);
  }

  return Result_state::State_Ok;
}

Result_state Viewer::Start(){
  // set initial vehicle state by cmd
  // need to sleep, because advertised channel is not ready immediately
  // simple test shows a short delay of 80 ms or so
  ROS_INFO("Viewer resetting vehicle state, sleeping for 1000 ms ...");
  ros::Duration(1.0).sleep();

  timer_ = AdapterManager::CreateTimer(ros::Duration(viewer_conf_.viewer_period()), 
                                                              &Viewer::OnTimer,
                                                              this);
  ROS_INFO("Viewer init done!");
  ROS_INFO("Viewer started");
  return Result_state::State_Ok;
}

void Viewer::RegisterAgents(const ViewerConf *viewer_conf_) {
  for (auto active_viewer_agent : viewer_conf_->active_viewer_agents()) {
    switch (active_viewer_agent) {
      case ViewerConf::LOCALIZATION_AGENT:
        agent_list_.emplace_back(std::move(new LocalizationAgent(ego_vehicle_odometry_, ego_vehicle_marker_)));
        break;
      case ViewerConf::PLANNING_AGENT:
        // agent_list_.emplace_back(std::move(new LonController()));
        break;
      case ViewerConf::CONTROL_AGENT:
        // agent_list_.emplace_back(std::move(new LonController()));
        break;
      case ViewerConf::PERCEPTION_AGENT:
        agent_list_.emplace_back(std::move(new PerceptionAgent(objects_, objects_marker_array_)));
        break;
      default:
        EERROR << "    Unknown active controller type: " << active_viewer_agent;
    }
  }
}

void Viewer::Stop() {
  
}

void Viewer::ProcessData() {
  for(auto &agent : agent_list_) {
    agent->ProcessData();
  }
}

void Viewer::OnTimer(const ros::TimerEvent &) {
  ros::Time begin = ros::Time::now();
  Result_state state = CheckInput();

  ProcessData();

  // for (auto &viwer : viewer_list_) {
  //   ros::Time start_timestamp = ros::Time::now();
  //   viwer->InterfaceMatch();
  //   ros::Time end_timestamp = ros::Time::now();
  // }
  
  /* init send data */
  ::viewer::VisualizingData visualizing_data;
  visualization_msgs::MarkerArray viewer_Objects;
  visualization_msgs::Marker viewer_vehicle;

  Publish(&objects_marker_array_);
}

void Viewer::Publish(visualization_msgs::MarkerArray *objects_marker_array) {
  // AdapterManager::PublishViewerVehicle(*viewer_vehicle_data);
  AdapterManager::PublishViewerObjects(*objects_marker_array);
  objects_marker_array_.markers.clear();
  AdapterManager::PublishViewerPath(trajectory_path_);
  AdapterManager::PublishViewerLocalization(ego_vehicle_marker_);
  // AdapterManager::PublishViewer(*visualizing_data);
}

} // namespace viewer
} // namespace EDrive