/******************************************************************************
 * Copyright 2022 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#include "control/src/control.h"

#include "common/src/adapters/adapter_manager.h"
#include "common/src/util/file.h"
#include "common/src/vehicle_state/vehicle_state_provider.h"

namespace EDrive {
namespace control {

using EDrive::Result_state;
using EDrive::common::adapter::AdapterManager;
using EDrive::common::VehicleStateProvider;

std::string Control::Name() const { return "EDrive_control"; }

Result_state Control::Init(){
  ROS_INFO("Control init, starting...");

  root_path = EDrive::common::util::GetRootPath();
  adapter_conf_file = root_path + adapter_conf_file;
  control_conf_file = root_path + control_conf_file;

  ROS_INFO("  registering node: %s", Name().c_str());
  AdapterManager::Init(adapter_conf_file);

  ROS_INFO("  controller init, starting...");
  EDrive::common::util::GetProtoFromASIIFile(control_conf_file, &control_conf_);
  if(State_Ok != controller_agent_.Init(&control_conf_)) {
    ROS_ERROR("    controller agent init failed, stopping...");
  }

  return State_Ok;
}

Result_state Control::CheckInput() {
  AdapterManager::Observe();
  auto trajectory_adapter = AdapterManager::GetPlanning();
  trajectory_ = trajectory_adapter->GetLatestObserved();

  auto localization_adapter = AdapterManager::GetLocalization();
  localization_ = localization_adapter->GetLatestObserved();

  VehicleStateProvider::instance()->Update(localization_);

  return State_Ok;
}

Result_state Control::Start(){
  // set initial vehicle state by cmd
  // need to sleep, because advertised channel is not ready immediately
  // simple test shows a short delay of 80 ms or so
  ROS_INFO("Control resetting vehicle state, sleeping for 1000 ms ...");
  ros::Duration(1.0).sleep();

  timer_ = common::adapter::AdapterManager::CreateTimer(ros::Duration(control_conf_.control_period()), 
                                                                &Control::OnTimer,
                                                                this);
  ROS_INFO("Control init done!");
  ROS_INFO("Control started");
  return State_Ok;
}

void Control::OnTimer(const ros::TimerEvent &) {
  Result_state status = CheckInput();

  ros::Time start_timestamp = ros::Time::now();
  ::control::CarlaEgoVehicleControl control_command;
  status = ProduceControlCommand(&control_command);
  ros::Time end_timestamp = ros::Time::now();
  SendCmd(&control_command);
}

Result_state Control::ProduceControlCommand(::control::CarlaEgoVehicleControl *control_command) {
  if(State_Ok != controller_agent_.ComputeControlCommand(&trajectory_, &localization_, control_command)) {
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