/******************************************************************************
 * Copyright 2023 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <ros/ros.h>

#include "control/controller/controller.h"
#include "control/proto/control_conf.pb.h"
#include "control/CarlaEgoVehicleControl.h"

#include "planning/ADCTrajectory.h"

namespace EDrive {
namespace control {

/**
 * @class ControllerAgent
 *
 * @brief manage all controllers declared in control config file.
 */
class ControllerAgent {
 public:
  /**
   * @brief initialize ControllerAgent
   * @param control_conf control configurations
   * @return Status initialization status
   */
  common::Result_state Init(const ControlConf *control_conf_);

  common::Result_state ComputeControlCommand(
      const ::planning::ADCTrajectory *trajectory, 
      const nav_msgs::Odometry *localization,
      ::control::CarlaEgoVehicleControl *control_command);

 private:
  /**
   * @brief
   * Register new controllers. If you need to add a new type of controller,
   * You should first register your controller in this function.
   */
  void RegisterControllers(const ControlConf *control_conf);

  std::vector<std::unique_ptr<Controller>> controller_list_;
};

} // namespace control
} // namespcae EDrive