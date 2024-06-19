/******************************************************************************
 * Copyright 2024 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#include "control/src/controller/mpc_controller.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <string>
#include <utility>
#include <vector>

#include "eigen3/Eigen/LU"

namespace EDrive {
namespace control {

using EDrive::common::Result_state;
using EDrive::common::VehicleStateProvider;
using Matrix = Eigen::MatrixXd;

namespace {

std::string GetLogFileName() {
  time_t raw_time;
  char name_buffer[80];
  std::time(&raw_time);
  strftime(name_buffer, 80, "/tmp/mpc_controller_%F_%H%M%S.csv",
           localtime(&raw_time));
  return std::string(name_buffer);
}

void WriteHeaders(std::ofstream &file_stream) {}
}  // namespace

MPCController::MPCController() : name_("MPC Controller") {

}

MPCController::~MPCController() {}

bool MPCController::LoadControlConf(const ControlConf *control_conf) {
  return true;
}

Result_state MPCController::Init(const ControlConf *control_conf) {
  return Result_state::State_Ok;
}

Result_state MPCController::Reset() {
  previous_heading_error_ = 0.0;
  previous_lateral_error_ = 0.0;
  return Result_state::State_Ok;
}

}  // namespace control
}  // namespace EDrive