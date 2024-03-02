/******************************************************************************
 * Copyright 2024 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include "control/src/controller/controller.h"

#include "control/src/common/trajectory_analyzer.h"

namespace EDrive {
namespace control {

using EDrive::Result_state;

/**
 * @class LatController
 *
 * @brief Lateral controller, to steer.
 */
class LatController : public Controller {
 public:
  /**
   * @brief constructor
   */
  LatController();

  /**
   * @brief destructor
   */
  virtual ~LatController();

  /**
   * @brief initialize Lateral Controller
   * @param control_conf control configurations
   * @return Status initialization status
   */
  Result_state Init(const ControlConf *control_conf) override;

  /**
   * @brief compute steering target based on current vehicle status
   *        and target trajectory
   * @param localization vehicle location
   * @param chassis vehicle status e.g., speed, acceleration
   * @param trajectory trajectory generated by planning
   * @param cmd control command
   * @return Status computation status
   */
  Result_state ComputeControlCommand(
      const ::planning::ADCTrajectory *trajectory,
      const nav_msgs::Odometry *localization,
      ::control::CarlaEgoVehicleControl *control_command) override;

  /**
   * @brief reset Lateral Controller
   * @return Status reset status
   */
  Result_state Reset() override;

  /**
   * @brief stop Lateral controller
   */
  void Stop() override;

  /**
   * @brief Lateral controller name
   * @return string controller name in string
   */
  std::string Name() const override;

  void ComputeLateralErrors(const TrajectoryAnalyzer *trajectory_analyzer);

 private:
  std::string name_;
  std::unique_ptr<TrajectoryAnalyzer> trajectory_analyzer_;
  const planning::ADCTrajectory *trajectory_message_ = nullptr;

  bool controller_initialized_ = false;
  const ControlConf *control_conf_ = nullptr;
};

} // namespace control
} // namespace EDrive