/******************************************************************************
 * Copyright 2023 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#include <ros/ros.h>
#include "control/src/controller/lon_controller.h"

namespace EDrive {
namespace control {

using EDrive::Result_state;

constexpr double GRA_ACC = 9.8;

LonController::LonController() : name_("LonController"){
  ROS_INFO("    Registering Lon controller...");
}

LonController::~LonController() {
  
}

Result_state LonController::Init(const ControlConf *control_conf) {
  control_conf_ = control_conf;
  if (control_conf_ == nullptr) {
    controller_initialized_ = false;
    ROS_ERROR("get_longitudinal_param() nullptr");
    return State_Failed;
  }

  const LonControllerConf &lon_controller_conf =
      control_conf_->lon_controller_conf();

  station_pid_controller_.Init(lon_controller_conf.station_pid_conf());
  speed_pid_controller_.Init(lon_controller_conf.low_speed_pid_conf());
  
  return State_Ok;
}

std::string LonController::Name() const { return name_; }

Result_state LonController::ComputeControlCommand() {
  // ComputeLongitudinalErrors(trajectory_analyzer_.get());
  return State_Ok;
}

Result_state LonController::Reset(){
  return State_Ok;
}

void LonController::Stop() {
  ROS_INFO("stop");
}

void LonController::ComputeLongitudinalErrors(const TrajectoryAnalyzer *trajectory_analyzer) {
  double s_matched = 0.0;
  double s_dot_matched = 0.0;
  double d_matched = 0.0;
  double d_dot_matched = 0.0;

  double x;
  double y;
}

} // namespace control
} // namespace EDrive