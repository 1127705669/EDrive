/******************************************************************************
 * Copyright 2022 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <string>
#include <ros/ros.h>

// #include "control/proto/control_cmd.pb.h"

#include "common/src/EDrive.h"

#include "controller/controller_agent.h"

#include "control/ControlCommand.h"

#include "control/CarlaEgoVehicleControl.h"

#include "control/proto/control_conf.pb.h"

#include "planning/ADCTrajectory.h"

namespace EDrive {
namespace control {

using EDrive::Result_state;

class Control : public EDrive::common::EDriveApp {
 public:

  std::string Name() const override;

  Result_state Init() override;

  Result_state Start() override;

  void Stop() override;

  virtual ~Control() = default;

 private:

 // Watch dog timer
  void OnTimer(const ros::TimerEvent &);

  EDrive::Result_state CheckInput();

  void SendCmd(::control::CarlaEgoVehicleControl *control_command);

  Result_state ProduceControlCommand(::control::CarlaEgoVehicleControl *control_command);
  
  ros::Timer timer_;
  ros::Time init_time_;
  ControllerAgent controller_agent_;
  ControlConf control_conf_;
  planning::ADCTrajectory trajectory_;

  std::string root_path;
  std::string adapter_conf_file = "/src/control/conf/adapter.conf";
  std::string control_conf_file = "/src/control/conf/control.conf";
  
};

} // namespace control
} // namespace EDrive