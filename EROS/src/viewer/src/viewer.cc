/******************************************************************************
 * Copyright 2022 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#include "ros/ros.h"
#include "viewer.h"

namespace EDrive {
namespace viewer {

// Planning::~Planning() { Stop(); }

std::string Viewer::Name() const { return "viewer"; }

EDrive::Result_state Viewer::Init(){
  return State_Ok;
}

EDrive::Result_state Viewer::Start(){
  return State_Ok;
}

void Viewer::Stop() {
  
}

} // viewer
} // EDrive