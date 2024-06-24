/******************************************************************************
 * Copyright 2023 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <cmath>
#include <string>
#include "common/src/state.h"

#include "control/proto/control_cmd.pb.h"
#include "control/proto/control_conf.pb.h"
#include "control/CarlaEgoVehicleControl.h"
#include "planning/ADCTrajectory.h"
#include "nav_msgs/Odometry.h"

/**
 * @namespace EDrive::control
 * @brief EDrive::control
 */
namespace EDrive {
namespace control {

/**
 * @class Controller
 *
 * @brief base class for all controllers.
 */
class Controller {
 public:
  /**
   * @brief constructor
   */
  Controller() = default;

  /**
   * @brief destructor
   */
  virtual ~Controller() = default;

  /**
   * @brief initialize Controller
   * @param control_conf control configurations
   * @return Status initialization status
   */
  virtual common::Result_state Init(const ControlConf *control_conf) = 0;

  /**
   * @brief compute control command based on current vehicle status
   *        and target trajectory
   * @param localization vehicle location
   * @param chassis vehicle status e.g., speed, acceleration
   * @param trajectory trajectory generated by planning
   * @param cmd control command
   * @return Status computation status
   */
  virtual common::Result_state ComputeControlCommand(
      const ::planning::ADCTrajectory *trajectory, 
      const nav_msgs::Odometry *localization, 
      control::ControlCommand *control_command) = 0;

  /**
   * @brief reset Controller
   * @return Status reset status
   */
  virtual common::Result_state Reset() = 0;

  /**
   * @brief controller name
   * @return string controller name in string
   */
  virtual std::string Name() const = 0;

  /**
   * @brief stop controller
   */
  virtual void Stop() = 0;
};

} // namespace control
} // namespace EDrive