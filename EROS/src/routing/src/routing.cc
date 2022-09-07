/******************************************************************************
 * Copyright 2022 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#include "ros/ros.h"
#include "EROS/src/routing/src/routing.h"

namespace EDrive {
namespace routing {

// Planning::~Planning() { Stop(); }

std::string Routing::Name() const { return "routing"; }

EDrive::common::Status Routing::Init(){

}

EDrive::common::Status Routing::Start(){
    
}

void Routing::Stop() {
  
}

} // routing
} // EDrive