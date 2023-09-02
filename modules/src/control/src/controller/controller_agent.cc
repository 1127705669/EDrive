/******************************************************************************
 * Copyright 2023 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#include "control/src/controller/lon_controller.h"

#include "control/src/controller/controller_agent.h"

#include "common/src/util/file.h"

namespace EDrive {
namespace control {

void ControllerAgent::RegisterControllers(const ControlConf *control_conf) {
  ROS_INFO("    Only Lon controllers as of now");
  for (auto active_controller : control_conf->active_controllers()) {
    switch (active_controller) {
      case ControlConf::LON_CONTROLLER:
        controller_list_.emplace_back(std::move(new LonController()));
        break;
      default:
        ROS_ERROR("    Unknown active controller type: ");
    }
  }
}

Result_state ControllerAgent::Init(const ControlConf *control_conf_) {
  RegisterControllers(control_conf_);
  for(auto &controller : controller_list_) {
    if (controller == NULL || EDrive::State_Ok != controller->Init()) {
      ROS_ERROR("    controller init failed!");
    }
  }
  return State_Ok;
}

Result_state ControllerAgent::ComputeControlCommand() {
  for (auto &controller : controller_list_) {
    ros::Time start_timestamp = ros::Time::now();
    controller->ComputeControlCommand();
    ros::Time end_timestamp = ros::Time::now();
  }
  return State_Ok;
}

} // namespace control
} // namespcae EDrive