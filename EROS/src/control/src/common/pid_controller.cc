/******************************************************************************
 * Copyright 2023 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#include "ros/ros.h"

#include <cmath>

#include "pid_controller.h"

namespace EDrive {
namespace control {

void PIDController::Init() {
  ROS_INFO("pid controller init...");
}

} // namespace control
} // namespace EDrive