/******************************************************************************
 * Copyright 2023 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#include "lon_controller.h"

namespace EDrive {
namespace control {

using EDrive::Result_state;

Result_state LonController::Init() {
  return State_Ok;
}

std::string LonController::Name() const { return name_; }

Result_state LonController::ComputeControlCommand() {
  return State_Ok;
}

Result_state LonController::Reset(){
  return State_Ok;
}

void LonController::Stop() {
  ROS_INFO("stop");
}

} // namespace control
} // namespace EDrive