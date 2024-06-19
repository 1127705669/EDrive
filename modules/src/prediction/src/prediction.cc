/******************************************************************************
 * Copyright 2022 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#include "ros/ros.h"
#include "prediction.h"

namespace EDrive {
namespace prediction {

using EDrive::common::Result_state;

std::string Prediction::Name() const { return "prediction"; }

Result_state Prediction::Init(){
  return Result_state::State_Ok;
}

Result_state Prediction::Start(){
  return Result_state::State_Ok;
}

void Prediction::Stop() {
  
}

} // prediction
} // EDrive