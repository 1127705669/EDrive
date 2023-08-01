/******************************************************************************
 * Copyright 2022 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <string>
#include "app/EDrive.h"
#include "app/state.h"

namespace EDrive {
namespace prediction {

class Prediction : public EDrive::common::EDriveApp {
 public:
//   Planning() = default;
//   virtual ~Planning();

  std::string Name() const override;

  EDrive::Result_state Init() override;

  EDrive::Result_state Start() override;

  void Stop() override;
};

} // prediction
} // EDrive