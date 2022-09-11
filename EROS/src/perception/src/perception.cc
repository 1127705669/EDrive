/******************************************************************************
 * Copyright 2022 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#include "ros/ros.h"
#include "EROS/src/perception/src/perception.h"

namespace EDrive {
namespace perception {

// Planning::~Planning() { Stop(); }

std::string Perception::Name() const { return "perception"; }

EDrive::common::Status Perception::Init(){

}

EDrive::common::Status Perception::Start(){
    
}

void Perception::Stop() {
  
}

} // perception
} // EDrive