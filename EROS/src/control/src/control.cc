/******************************************************************************
 * Copyright 2022 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#include "ros/ros.h"

#include "app/adapters/adapter_manager.h"

#include "control.h"

namespace EDrive {
namespace control {

using EDrive::Result_state;

std::string Control::Name() const { return "control"; }

Result_state Control::Init(){
  if(State_Ok != controller_agent_.Init()) {
    ROS_INFO("controller agent init failed, stopping...");
  }
  return State_Ok;
}

Result_state Control::Start(){
  timer_ = common::adapter::AdapterManager::CreateTimer(ros::Duration(control_period), 
                                                                &Control::OnTimer,
                                                                this);
  return State_Ok;
}

void Control::OnTimer(const ros::TimerEvent &) {
  ros::Time start_timestamp = ros::Time::now();
  Result_state status = ProduceControlCommand();
  ros::Time end_timestamp = ros::Time::now();
}

Result_state Control::ProduceControlCommand() {
  if(State_Ok != controller_agent_.ComputeControlCommand()) {
    ROS_INFO("controller agent compute control command failed, stopping...");
  }
  return State_Ok;
}

void Control::Stop() {

}

void Control::SendCmd() {
  
}

} // namespace control
} // namespace EDrive