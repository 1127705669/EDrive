/******************************************************************************
 * Copyright 2024 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#include <ros/ros.h>
#include "control/src/controller/lat_controller.h"

namespace EDrive {
namespace control {

using EDrive::Result_state;

constexpr double GRA_ACC = 9.8;

LatController::LatController() : name_("LatController"){
  ROS_INFO("    registering Lat controller...");
}

LatController::~LatController() {
  
}

Result_state LatController::Init(const ControlConf *control_conf) {

  return State_Ok;
}

std::string LatController::Name() const { return name_; }

Result_state LatController::ComputeControlCommand(
    const ::planning::ADCTrajectory *trajectory,
    const nav_msgs::Odometry *localization,
    ::control::CarlaEgoVehicleControl *control_command) {
  trajectory_message_ = trajectory;
  if (trajectory_analyzer_ == nullptr) {
    trajectory_analyzer_.reset(new TrajectoryAnalyzer(trajectory_message_));
  }

  ComputeLateralErrors(trajectory_analyzer_.get());

  return State_Ok;
}

Result_state LatController::Reset(){
  return State_Ok;
}

void LatController::Stop() {
  ROS_INFO("stop");
}

void LatController::ComputeLateralErrors(const TrajectoryAnalyzer *trajectory_analyzer) {
  
}

} // namespace control
} // namespace EDrive