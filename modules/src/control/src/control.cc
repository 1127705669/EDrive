/******************************************************************************
 * Copyright 2022 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#include "common/src/adapters/adapter_manager.h"

#include "control/src/control.h"

#include "common/src/util/file.h"

namespace EDrive {
namespace control {

using EDrive::Result_state;
using EDrive::common::adapter::AdapterManager;

std::string Control::Name() const { return "control"; }

Result_state Control::Init(){
  ROS_INFO("Control init, starting...");

  root_path = EDrive::common::util::GetRootPath();
  adapter_conf_file = root_path + adapter_conf_file;
  control_conf_file = root_path + control_conf_file;

  ROS_INFO("  registering node: %s", Name().c_str());
  AdapterManager::Init(adapter_conf_file);

  EDrive::common::util::GetProtoFromASIIFile(control_conf_file, &control_conf_);
  ROS_INFO("  controller init, start...");
  if(State_Ok != controller_agent_.Init()) {
    ROS_ERROR("    controller agent init failed, stopping...");
  }

  return State_Ok;
}

Result_state Control::Start(){
  timer_ = common::adapter::AdapterManager::CreateTimer(ros::Duration(control_conf_.control_period()), 
                                                                &Control::OnTimer,
                                                                this);
  ROS_INFO("Control init done!");
  ROS_INFO("Control started");
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