/******************************************************************************
 * Copyright 2022 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#include <ros/ros.h>
#include "planning/src/planning.h"
#include "common/src/util/file.h"

namespace EDrive {
namespace planning {

using EDrive::common::Result_state;
using EDrive::common::adapter::AdapterManager;
using ::planning::ADCTrajectory;

Planning::~Planning() { Stop(); }

std::string Planning::Name() const { return "EDrive_planning"; }

void Planning::PublishPlanningPb(ADCTrajectory* trajectory_pb) {
  Publish(trajectory_pb);
}

Result_state Planning::RegisterPlanners() {

  return Result_state::State_Ok;
}

void Planning::RunOnce() {
  Result_state state;

  // snapshot all coming data
  AdapterManager::Observe();
  
  state = RegisterPlanners();

  ADCTrajectory trajectory_pb;

  
  
  for(int i = 0; i < 180; i++) {
    ::common::TrajectoryPoint trajectory_point_;
    trajectory_point_.path_point.x = start_point_x + 0.5*i;
    trajectory_point_.path_point.y = start_point_y + 0.5*i;
    
    trajectory_pb.trajectory_point.push_back(trajectory_point_);
  }

  PublishPlanningPb(&trajectory_pb);
}

Result_state Planning::Init(){
  ROS_INFO("Planning init, starting...");

  root_path = EDrive::common::util::GetRootPath();
  adapter_conf_file = root_path + adapter_conf_file;
  planning_conf_file = root_path + planning_conf_file;

  if (!AdapterManager::Initialized()) {
    ROS_INFO("  registering node: %s", Name().c_str());
    AdapterManager::Init(adapter_conf_file);
  }

  ROS_INFO("Planning init done!");
  ROS_INFO("Planning started");
  return Result_state::State_Ok;
}

Result_state Planning::Start(){
  timer_ = EDrive::common::adapter::AdapterManager::CreateTimer(ros::Duration(planning_period), 
                                                                &Planning::OnTimer,
                                                                this);
  return Result_state::State_Ok;
}

void Planning::CheckInput(){
  AdapterManager::Observe();
  auto position_adapter = AdapterManager::GetLocalization();
  position_ = position_adapter->GetLatestObserved();
}

void Planning::Stop() {
  
}

void Planning::OnTimer(const ros::TimerEvent &) {
  ros::Time begin = ros::Time::now();
  
  CheckInput();

  if(is_initialized){
    
  }else{
    start_point_x = position_.pose.pose.position.x - 2;
    start_point_y = position_.pose.pose.position.y - 2;
    ROS_INFO("%f     %f",start_point_x,start_point_y);
    is_initialized = true;
  }
  
  RunOnce();
}

} // namespace planning
} // namespace EDrive