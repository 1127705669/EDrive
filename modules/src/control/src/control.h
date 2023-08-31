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

  void SendCmd(::control::CarlaEgoVehicleControl *control_command);

  Result_state ProduceControlCommand(::control::CarlaEgoVehicleControl *control_command);
  
  ros::Timer timer_;
  const float control_period = 0.01;
  ros::Time init_time_;
  ControllerAgent controller_agent_;
  
};

} // namespace control
} // namespace EDrive