/******************************************************************************
 * Copyright 2022 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#include "ros/ros.h"
#include "perception.h"

namespace EDrive {
namespace perception {

// Planning::~Planning() { Stop(); }

std::string Perception::Name() const { return "perception"; }

EDrive::Result_state Perception::Init(){
  return State_Ok;
}

EDrive::Result_state Perception::Start(){
  return State_Ok;
}

void Perception::Stop() {
  
}

} // perception
} // EDrive