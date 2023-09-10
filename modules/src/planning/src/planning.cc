/******************************************************************************
 * Copyright 2022 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#include <ros/ros.h>
#include "planning/src/planning.h"
#include "common/src/util/file.h"

namespace EDrive {
namespace planning {

using EDrive::Result_state;
using ::planning::ADCTrajectory;
using EDrive::common::adapter::AdapterManager;

Planning::~Planning() { Stop(); }

std::string Planning::Name() const { return "planning"; }

void Planning::PublishPlanningPb(ADCTrajectory* trajectory_pb) {
  Publish(trajectory_pb);
}

Result_state Planning::RegisterPlanners() {

  return State_Ok;
}

void Planning::RunOnce() {
  Result_state state;

  // snapshot all coming data
  AdapterManager::Observe();
  
  state = RegisterPlanners();

  ADCTrajectory trajectory_pb;
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

  return State_Ok;
}

Result_state Planning::Start(){
  timer_ = EDrive::common::adapter::AdapterManager::CreateTimer(ros::Duration(planning_period), 
                                                                &Planning::OnTimer,
                                                                this);
  return State_Ok;
}

void Planning::Stop() {
  
}

void Planning::OnTimer(const ros::TimerEvent &) {
  ros::Time begin = ros::Time::now();
  RunOnce();
}

} // planning
} // EDrive