/******************************************************************************
 * Copyright 2022 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#include "ros/ros.h"

#include "app/adapters/adapter_manager.h"

#include "control.h"

namespace EDrive {
namespace control {

using EDrive::common::Status;

// Planning::~Planning() { Stop(); }

std::string Control::Name() const { return "control"; }

EDrive::Result_state Control::Init(){
  return State_Ok;
}

EDrive::Result_state Control::Start(){
  
  timer_ = EDrive::common::adapter::AdapterManager::CreateTimer(ros::Duration(control_period), 
                                                                &Control::OnTimer,
                                                                this);
  return State_Ok;
}

void Control::Stop() {
  
}

void Control::OnTimer(const ros::TimerEvent &) {
  ros::Time begin = ros::Time::now();
  ROS_INFO("10");
}

} // control
} // EDrive