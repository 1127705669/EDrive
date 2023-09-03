/******************************************************************************
 * Copyright 2023 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#include <ros/ros.h>

#include <cmath>

#include "control/src/common/pid_controller.h"

namespace EDrive {
namespace control {

void PIDController::Init(const PidConf &pid_conf) {
  ROS_INFO("      pid controller init, staring...");
  SetPID(pid_conf);
}

void PIDController::Reset() {
  previous_error_ = 0.0;
  previous_output_ = 0.0;
  integral_ = 0.0;
  first_hit_ = true;
  integrator_saturation_status_ = 0;
  output_saturation_status_ = 0;
}

void PIDController::SetPID(const PidConf &pid_conf) {
  kp_ = pid_conf.kp();
  ki_ = pid_conf.ki();
  kd_ = pid_conf.kd();
  kaw_ = pid_conf.kaw();
}

} // namespace control
} // namespace EDrive