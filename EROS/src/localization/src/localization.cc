/******************************************************************************
 * Copyright 2022 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#include "ros/ros.h"
#include "localization.h"

namespace EDrive {
namespace localization {

// Planning::~Planning() { Stop(); }

std::string Localization::Name() const { return "localization"; }

EDrive::Result_state Localization::Init(){
  return State_Ok;
}

EDrive::Result_state Localization::Start(){
  return State_Ok;
}

void Localization::Stop() {
  
}

} // localization
} // EDrive