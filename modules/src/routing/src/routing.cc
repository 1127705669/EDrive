/******************************************************************************
 * Copyright 2022 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#include "ros/ros.h"
#include "routing.h"

namespace EDrive {
namespace routing {

using EDrive::common::Result_state;
// Planning::~Planning() { Stop(); }

std::string Routing::Name() const { return "routing"; }

Result_state Routing::Init(){
  return Result_state::State_Ok;
}

Result_state Routing::Start(){
  return Result_state::State_Ok;
}

void Routing::Stop() {
  
}

} // routing
} // EDrive