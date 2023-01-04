/******************************************************************************
 * Copyright 2022 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#include "ros/ros.h"
#include "planning.h"

namespace EDrive {
namespace planning {

// Planning::~Planning() { Stop(); }

std::string Planning::Name() const { return "planning"; }

EDrive::Result_state Planning::Init(){
  return State_Ok;
}

EDrive::Result_state Planning::Start(){
  return State_Ok;
}

void Planning::Stop() {
  
}

} // planning
} // EDrive