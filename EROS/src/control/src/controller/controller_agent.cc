/******************************************************************************
 * Copyright 2023 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#include "controller_agent.h"

namespace EDrive {
namespace control {

Result_state ControllerAgent::Init() {
  return State_Ok;
}

Result_state ControllerAgent::ComputeControlCommand() {
  return State_Ok;
}

} // namespace control
} // namespcae EDrive