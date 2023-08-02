/******************************************************************************
 * Copyright 2022 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <string>
#include "common/src/EDrive.h"
#include "common/src/state.h"

namespace EDrive {
namespace perception {

class Perception : public EDrive::common::EDriveApp {
 public:
//   Planning() = default;
//   virtual ~Planning();

  std::string Name() const override;

  EDrive::Result_state Init() override;

  EDrive::Result_state Start() override;

  void Stop() override;
};

} // perception
} // EDrive