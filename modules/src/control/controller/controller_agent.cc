/******************************************************************************
 * Copyright 2023 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#include "control/controller/lon_controller.h"
#include "control/controller/lat_controller.h"
#include "control/controller/mpc_controller.h"

#include "control/controller/controller_agent.h"

#include "common/util/file.h"

namespace EDrive {
namespace control {

using EDrive::common::Result_state;

void ControllerAgent::RegisterControllers(const ControlConf *control_conf_) {
  ROS_INFO("    Only support MPC controller or Lat + Lon controllers as of now");
  for (auto active_controller : control_conf_->active_controllers()) {
    switch (active_controller) {
      case ControlConf::LON_CONTROLLER:
        controller_list_.emplace_back(std::move(new LonController()));
        break;
      case ControlConf::LAT_CONTROLLER:
        controller_list_.emplace_back(std::move(new LatController()));
        break;
      case ControlConf::MPC_CONTROLLER:
        controller_list_.emplace_back(std::move(new MPCController()));
        break;
      default:
        ROS_ERROR("    Unknown active controller type: ");
    }
  }
}

Result_state ControllerAgent::Init(const ControlConf *control_conf_) {
  RegisterControllers(control_conf_);
  for(auto &controller : controller_list_) {
    if (controller == NULL || Result_state::State_Ok != controller->Init(control_conf_)) {
      ROS_ERROR("    controller init failed!");
    }
  }
  return Result_state::State_Ok;
}

Result_state ControllerAgent::ComputeControlCommand(
    const ::planning::ADCTrajectory *trajectory, 
    const nav_msgs::Odometry *localization,
    ControlCommand *control_command) {
  for (auto &controller : controller_list_) {
    ros::Time start_timestamp = ros::Time::now();
    controller->ComputeControlCommand(trajectory, localization, control_command);
    ros::Time end_timestamp = ros::Time::now();
  }
  return Result_state::State_Ok;
}

} // namespace control
} // namespcae EDrive