/******************************************************************************
 * Copyright 2022 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#include "modules/common/status/status.h"
#include "modules/control/controller/lon_controller.h"

namespace EDrive {
namespace control {

using EDrive::common::Status;

constexpr double GRA_ACC = 9.8;

LonController::LonController() {

}

Status LonController::Init() {

}

void LonController::Stop() {

}

Status LonController::ComputeControlCommand() {

}

Status LonController::Reset() {

}

std::string LonController::Name() const { return name_; }

void LonController::ComputeLongitudinalErrors() {
    
}

} // control
} // EDrive