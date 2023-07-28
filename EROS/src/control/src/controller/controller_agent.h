/******************************************************************************
 * Copyright 2023 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <ros/ros.h>

#include "controller.h"

namespace control_msg = control;

namespace EDrive {
namespace control {

using EDrive::Result_state;

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
  Result_state Init();

  Result_state ComputeControlCommand(control_msg::ControlCommand controlcommand_);
 private:
  std::vector<std::unique_ptr<Controller>> controller_list_;
};

} // namespace control
} // namespcae EDrive