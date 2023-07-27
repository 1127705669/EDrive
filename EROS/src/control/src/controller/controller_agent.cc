/******************************************************************************
 * Copyright 2023 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#include "lon_controller.h"

#include "controller_agent.h"

namespace EDrive {
namespace control {

Result_state ControllerAgent::Init() {
  controller_list_.emplace_back(new LonController());
  for(auto &controller : controller_list_) {
    if (controller == NULL || EDrive::State_Ok != controller->Init()) {
      ROS_ERROR("controller init failed!");
    }
  }
  return State_Ok;
}

Result_state ControllerAgent::ComputeControlCommand(control_msg::ControlCommand controlcommand_) {
  for (auto &controller : controller_list_) {
    ros::Time start_timestamp = ros::Time::now();
    controller->ComputeControlCommand(controlcommand_);
    ros::Time end_timestamp = ros::Time::now();
  }
  return State_Ok;
}

} // namespace control
} // namespcae EDrive