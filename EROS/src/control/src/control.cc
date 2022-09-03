/******************************************************************************
 * Copyright 2022 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#include "ros/ros.h"
#include "EROS/src/control/src/control.h"

namespace EDrive {
namespace control {

// Planning::~Planning() { Stop(); }

std::string Control::Name() const { return "control"; }

EDrive::common::Status Control::Init(){

}

EDrive::common::Status Control::Start(){
    
}

void Control::Stop() {
  
}

} // control
} // EDrive