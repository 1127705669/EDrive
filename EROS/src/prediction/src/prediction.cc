/******************************************************************************
 * Copyright 2022 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#include "ros/ros.h"
#include "EROS/src/prediction/src/prediction.h"

namespace EDrive {
namespace prediction {

// Planning::~Planning() { Stop(); }

std::string Prediction::Name() const { return "prediction"; }

} // prediction
} // EDrive