/******************************************************************************
 * Copyright 2023 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include "ros/ros.h"
#include "controller.h"

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

  Result_state ComputeControlCommand();

};

} // namespace control
} // namespcae EDrive