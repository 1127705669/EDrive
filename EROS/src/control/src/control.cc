/******************************************************************************
 * Copyright 2022 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#include "ros/ros.h"
#include "control.h"

namespace EDrive {
namespace control {

using EDrive::common::Status;

// Planning::~Planning() { Stop(); }

std::string Control::Name() const { return "control"; }

EDrive::Result_state Control::Init(){

  return State_Ok;
}

EDrive::Result_state Control::Start(){
  return State_Ok;
}

void Control::Stop() {
  
}

} // control
} // EDrive