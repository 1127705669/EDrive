/******************************************************************************
 * Copyright 2022 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#include "ros/ros.h"
#include "prediction.h"

namespace EDrive {
namespace prediction {

// Planning::~Planning() { Stop(); }

std::string Prediction::Name() const { return "prediction"; }

EDrive::Result_state Prediction::Init(){
  return State_Ok;
}

EDrive::Result_state Prediction::Start(){
  return State_Ok;
}

void Prediction::Stop() {
  
}

} // prediction
} // EDrive