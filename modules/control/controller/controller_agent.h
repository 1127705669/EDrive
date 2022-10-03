/******************************************************************************
 * Copyright 2022 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <memory>
#include <vector>

#include "ros/include/ros/ros.h"

#include "modules/control/controller/controller.h"

/**
 * @namespace EDrive::control
 * @brief EDrive::control
 */
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
  common::Status Init(const ControlConf *control_conf);


};

} // control
} // EDrive