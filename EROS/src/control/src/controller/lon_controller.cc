/******************************************************************************
 * Copyright 2023 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#include "lon_controller.h"

namespace EDrive {
namespace control {

using EDrive::Result_state;

constexpr double GRA_ACC = 9.8;

LonController::LonController() : name_("LonController"){

}

LonController::~LonController() {
  
}

Result_state LonController::Init() {
  return State_Ok;
}

std::string LonController::Name() const { return name_; }

Result_state LonController::ComputeControlCommand() {
  ComputeLongitudinalErrors();
  return State_Ok;
}

Result_state LonController::Reset(){
  return State_Ok;
}

void LonController::Stop() {
  ROS_INFO("stop");
}

void LonController::ComputeLongitudinalErrors() {
  double s_matched = 0.0;
  double s_dot_matched = 0.0;
  double d_matched = 0.0;
  double d_dot_matched = 0.0;
  
}

} // namespace control
} // namespace EDrive