/******************************************************************************
 * Copyright 2022 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#include <ros/ros.h>
#include "common/src/adapters/adapter_manager.h"
#include "planning.h"

namespace EDrive {
namespace planning {

// Planning::~Planning() { Stop(); }

std::string Planning::Name() const { return "planning"; }

EDrive::Result_state Planning::Init(){
  return State_Ok;
}

EDrive::Result_state Planning::Start(){
  timer_ = EDrive::common::adapter::AdapterManager::CreateTimer(ros::Duration(planning_period), 
                                                                &Planning::OnTimer,
                                                                this);
  return State_Ok;
}

void Planning::Stop() {
  
}

void Planning::OnTimer(const ros::TimerEvent &) {
  ros::Time begin = ros::Time::now();
  ROS_INFO("10");
}

} // planning
} // EDrive