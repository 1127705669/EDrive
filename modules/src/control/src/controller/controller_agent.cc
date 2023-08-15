/******************************************************************************
 * Copyright 2023 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#include "control/src/controller/lon_controller.h"

#include "control/src/controller/controller_agent.h"

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