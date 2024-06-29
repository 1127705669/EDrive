/******************************************************************************
 * Copyright 2022 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <string>
#include <ros/ros.h>

// #include "control/proto/control_cmd.pb.h"

#include "common/src/EDrive.h"

#include "control/controller/controller_agent.h"

#include "control/proto/control_cmd.pb.h"

#include "control/CarlaEgoVehicleControl.h"

#include "control/proto/control_conf.pb.h"

#include "planning/ADCTrajectory.h"

#include <nav_msgs/Odometry.h>

namespace EDrive {
namespace control {

class Control : public EDrive::common::EDriveApp {
 public:

  std::string Name() const override;

  common::Result_state Init() override;

  common::Result_state Start() override;

  void Stop() override;

  virtual ~Control() = default;

 private:

 // Watch dog timer
  void OnTimer(const ros::TimerEvent &);

  common::Result_state CheckInput();

  void SendCmd(::control::CarlaEgoVehicleControl *control_command);

  common::Result_state ProduceControlCommand(ControlCommand *control_command);

  void ConvertControlCommandToSimulator(
    const ControlCommand& control_command,
    ::control::CarlaEgoVehicleControl& simulator_control_command);

  ros::Timer timer_;
  ros::Time init_time_;
  ControllerAgent controller_agent_;
  ControlConf control_conf_;
  planning::ADCTrajectory trajectory_;
  nav_msgs::Odometry localization_; 

  std::string root_path;
  std::string adapter_conf_file = "/src/control/conf/adapter.conf";
  std::string control_conf_file = "/src/control/conf/control.conf";

  std::shared_ptr<DependencyInjector> injector_ = nullptr;

};

} // namespace control
} // namespace EDrive