/******************************************************************************
 * Copyright 2022 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#include "ros/ros.h"
#include "routing.h"

namespace EDrive {
namespace routing {

// Planning::~Planning() { Stop(); }

std::string Routing::Name() const { return "routing"; }

EDrive::Result_state Routing::Init(){
  return State_Ok;
}

EDrive::Result_state Routing::Start(){
  return State_Ok;
}

void Routing::Stop() {
  
}

} // routing
} // EDrive