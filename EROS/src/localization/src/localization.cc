/******************************************************************************
 * Copyright 2022 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#include "ros/ros.h"
#include "EROS/src/localization/src/localization.h"

namespace EDrive {
namespace localization {

// Planning::~Planning() { Stop(); }

std::string Localization::Name() const { return "localization"; }

EDrive::common::Status Localization::Init(){

}

EDrive::common::Status Localization::Start(){
    
}

void Localization::Stop() {
  
}

} // localization
} // EDrive