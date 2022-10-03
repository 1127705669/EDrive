/******************************************************************************
 * Copyright 2022 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <string>
#include <vector>

#include "modules/common/data/error_code.h"
#include "modules/control/controller/controller.h"

namespace EDrive {
namespace control {

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
  common::Status Init() override;

  /**
   * @brief compute brake / throttle values based on current vehicle status
   *        and target trajectory
   * @param localization vehicle location
   * @param chassis vehicle status e.g., speed, acceleration
   * @param trajectory trajectory generated by planning
   * @param cmd control command
   * @return Status computation status
   */
  common::Status ComputeControlCommand() override;

  /**
   * @brief reset longitudinal controller
   * @return Status reset status
   */
  common::Status Reset() override;

  /**
   * @brief stop longitudinal controller
   */
  void Stop() override;

  /**
   * @brief longitudinal controller name
   * @return string controller name in string
   */
  std::string Name() const override;
 protected:
  void ComputeLongitudinalErrors();

 private:
  std::string name_;
  bool controller_initialized_ = false;
};

} // control
} // EDrive