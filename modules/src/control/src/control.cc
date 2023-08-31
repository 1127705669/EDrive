/******************************************************************************
 * Copyright 2022 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#include "common/src/adapters/adapter_manager.h"

#include "control/src/control.h"

namespace EDrive {
namespace control {

using EDrive::Result_state;
using EDrive::common::adapter::AdapterManager;

std::string Control::Name() const { return "control"; }

Result_state Control::Init(){
  if(State_Ok != controller_agent_.Init()) {
    ROS_INFO("controller agent init failed, stopping...");
  }
  std::string control_name = "/home/ethan/workset/EDrive/modules/src/control/conf/adapter.conf";
  AdapterManager::Init(control_name);

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
  ::control::CarlaEgoVehicleControl control_command;
  Result_state status = ProduceControlCommand(&control_command);
  ros::Time end_timestamp = ros::Time::now();
  SendCmd(&control_command);
}

Result_state Control::ProduceControlCommand(::control::CarlaEgoVehicleControl *control_command) {
  if(State_Ok != controller_agent_.ComputeControlCommand()) {
    ROS_INFO("controller agent compute control command failed, stopping...");
  }
  return State_Ok;
}

void Control::Stop() {

}

void Control::SendCmd(::control::CarlaEgoVehicleControl *control_command) {
  AdapterManager::PublishControlCommand(*control_command);
}

} // namespace control
} // namespace EDrive