/******************************************************************************
 * Copyright 2022 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#include "ros/ros.h"
#include "EROS/src/control/src/control.h"

namespace EDrive {
namespace control {

using EDrive::common::Status;

// Planning::~Planning() { Stop(); }

std::string Control::Name() const { return "control"; }

Status Control::Init(){

  return Status::OK();
}

Status Control::Start(){
  return Status::OK();
}

void Control::Stop() {
  
}

} // control
} // EDrive