/******************************************************************************
 * Copyright 2022 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#include "control/src/control.h"

#include "common/adapters/adapter_manager.h"
#include "common/util/file.h"
#include "common/vehicle_state/vehicle_state_provider.h"

namespace EDrive {
namespace control {

using EDrive::common::Result_state;
using EDrive::common::adapter::AdapterManager;
using EDrive::common::VehicleStateProvider;

std::string Control::Name() const { return "EDrive_control"; }

Result_state Control::Init(){
  EINFO << "Control init, starting...";

  root_path = EDrive::common::util::GetRootPath();
  adapter_conf_file = root_path + adapter_conf_file;
  control_conf_file = root_path + control_conf_file;

  EINFO << "  registering node: " << Name().c_str();
  AdapterManager::Init(adapter_conf_file);

  EINFO << "  controller init, starting...";
  EDrive::common::util::GetProtoFromASCIIFile(control_conf_file, &control_conf_);
  if(Result_state::State_Ok != controller_agent_.Init(&control_conf_)) {
    EERROR << "    controller agent init failed, stopping...";
  }

  return Result_state::State_Ok;
}

Result_state Control::CheckInput() {
  AdapterManager::Observe();
  auto trajectory_adapter = AdapterManager::GetPlanning();
  trajectory_ = trajectory_adapter->GetLatestObserved();

  auto localization_adapter = AdapterManager::GetLocalization();
  localization_ = localization_adapter->GetLatestObserved();

  VehicleStateProvider::Instance()->Update(localization_);

  return Result_state::State_Ok;
}

Result_state Control::Start(){
  // set initial vehicle state by cmd
  // need to sleep, because advertised channel is not ready immediately
  // simple test shows a short delay of 80 ms or so
  EINFO << "Control resetting vehicle state, sleeping for 1000 ms ...";
  ros::Duration(1.0).sleep();

  timer_ = common::adapter::AdapterManager::CreateTimer(ros::Duration(control_conf_.control_period()), 
                                                                &Control::OnTimer,
                                                                this);
  EINFO << "Control init done!";
  EINFO << "Control started";
  return Result_state::State_Ok;
}

void Control::ConvertControlCommandToSimulator(
    const ControlCommand& control_command, 
    ::control::CarlaEgoVehicleControl& simulator_control_command) {
  simulator_control_command.steer = -control_command.steering_target()/100;
  // simulator_control_command.throttle = control_command.throttle()/100;
  // simulator_control_command.brake = control_command.brake()/100;
  // ROS_INFO("%f",simulator_control_command.throttle);
  simulator_control_command.throttle = 0.3;
}

void Control::OnTimer(const ros::TimerEvent &) {
  Result_state status = CheckInput();

  ros::Time start_timestamp = ros::Time::now();
  ::control::CarlaEgoVehicleControl simulator_control_command;
  ControlCommand control_command;
  status = ProduceControlCommand(&control_command);
  ros::Time end_timestamp = ros::Time::now();

  ConvertControlCommandToSimulator(control_command, simulator_control_command);
  SendCmd(&simulator_control_command);
}

Result_state Control::ProduceControlCommand(ControlCommand *control_command) {
  if(Result_state::State_Ok != controller_agent_.ComputeControlCommand(&trajectory_, &localization_, control_command)) {
    EINFO << "controller agent compute control command failed, stopping...";
  }
  return Result_state::State_Ok;
}

void Control::Stop() {

}

void Control::SendCmd(::control::CarlaEgoVehicleControl *control_command) {
  AdapterManager::PublishControlCommand(*control_command);
}

} // namespace control
} // namespace EDrive