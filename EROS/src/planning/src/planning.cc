/******************************************************************************
 * Copyright 2022 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#include "ros/ros.h"
#include "EROS/src/planning/src/planning.h"

namespace EDrive {
namespace planning {

// Planning::~Planning() { Stop(); }

std::string Planning::Name() const { return "planning"; }

EDrive::common::Status Planning::Init(){

}

EDrive::common::Status Planning::Start(){
    
}

void Planning::Stop() {
  
}

} // planning
} // EDrive