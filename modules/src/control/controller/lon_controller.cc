/******************************************************************************
 * Copyright 2023 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#include "common/src/log.h"

#include <ros/ros.h>
#include "control/controller/lon_controller.h"

using EDrive::common::VehicleStateProvider;

namespace EDrive {
namespace control {

using EDrive::common::Result_state;

constexpr double GRA_ACC = 9.8;

LonController::LonController() : name_("LonController"){
  EINFO << "    registering Lon controller...";
}

LonController::~LonController() {
  
}

Result_state LonController::Init(const ControlConf *control_conf) {
  control_conf_ = control_conf;
  if (control_conf_ == nullptr) {
    controller_initialized_ = false;
    EERROR << "get_longitudinal_param() nullptr";
    return Result_state::State_Failed;
  }

  const LonControllerConf &lon_controller_conf =
      control_conf_->lon_controller_conf();

  EINFO << "      station pid controller init, staring...";
  station_pid_controller_.Init(lon_controller_conf.station_pid_conf());
  EINFO << "      speed pid controller init, staring...";
  speed_pid_controller_.Init(lon_controller_conf.low_speed_pid_conf());
  
  return Result_state::State_Ok;
}

std::string LonController::Name() const { return name_; }

Result_state LonController::ComputeControlCommand(
    const ::planning::ADCTrajectory *trajectory,
    const nav_msgs::Odometry *localization,
    ControlCommand *control_command) {
  trajectory_message_ = trajectory;
  if (trajectory_analyzer_ == nullptr) {
    trajectory_analyzer_.reset(new TrajectoryAnalyzer(trajectory_message_));
  }
  
  const LonControllerConf &lon_controller_conf =
      control_conf_->lon_controller_conf();

  double brake_cmd = 0.0;
  double throttle_cmd = 0.0;
  double ts = lon_controller_conf.ts();

  ComputeLongitudinalErrors(trajectory_analyzer_.get());
  return Result_state::State_Ok;
}

Result_state LonController::Reset(){
  return Result_state::State_Ok;
}

void LonController::Stop() {
  EINFO << "stop";
}

void LonController::ComputeLongitudinalErrors(const TrajectoryAnalyzer *trajectory_analyzer) {
  double s_matched = 0.0;
  double s_dot_matched = 0.0;
  double d_matched = 0.0;
  double d_dot_matched = 0.0;

  double x;
  double y;
  auto matched_point = trajectory_analyzer->QueryMatchedPathPoint(
      VehicleStateProvider::Instance()->x(),
      VehicleStateProvider::Instance()->y());

  trajectory_analyzer->ToTrajectoryFrame(
      VehicleStateProvider::Instance()->x(),
      VehicleStateProvider::Instance()->y(),
      VehicleStateProvider::Instance()->heading(),
      VehicleStateProvider::Instance()->linear_velocity(), matched_point,
      &s_matched, &s_dot_matched, &d_matched, &d_dot_matched);
}

} // namespace control
} // namespace EDrive