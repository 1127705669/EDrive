/******************************************************************************
 * Copyright 2023 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include "control/src/common/pid_controller.h"
#include "control/src/common/trajectory_analyzer.h"

#include "control/src/controller/controller.h"

#include "planning/ADCTrajectory.h"

namespace EDrive {
namespace control {

using EDrive::Result_state;

/**
 * @class LonController
 *
 * @brief Longitudinal controller, to compute brake / throttle values.
 */
class LonController : public Controller {
 public:
  /**
   * @brief constructor
   */
  LonController();

  /**
   * @brief destructor
   */
  virtual ~LonController();

  /**
   * @brief initialize Longitudinal Controller
   * @param control_conf control configurations
   * @return Status initialization status
   */
  Result_state Init(const ControlConf *control_conf) override;

   /**
   * @brief compute control command based on current vehicle status
   *        and target trajectory
   * @param localization vehicle location
   * @param chassis vehicle status e.g., speed, acceleration
   * @param trajectory trajectory generated by planning
   * @param cmd control command
   * @return Status computation status
   */
  Result_state ComputeControlCommand(::control::CarlaEgoVehicleControl *control_command) override;

  /**
   * @brief reset Controller
   * @return Status reset status
   */
  Result_state Reset() override;

  /**
   * @brief controller name
   * @return string controller name in string
   */
  std::string Name() const override;

  /**
   * @brief stop controller
   */
  void Stop() override;

 protected:
  void ComputeLongitudinalErrors(const TrajectoryAnalyzer *trajectory_analyzer);

 private:
  std::string name_;
  PIDController speed_pid_controller_;
  PIDController station_pid_controller_;
  bool controller_initialized_ = false;
  const ControlConf *control_conf_ = nullptr;
  const planning::ADCTrajectory *trajectory_message_ = nullptr;

  std::unique_ptr<TrajectoryAnalyzer> trajectory_analyzer_;

};

} // namespace control
} // namespace EDrive