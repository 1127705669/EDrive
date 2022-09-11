/******************************************************************************
 * Copyright 2022 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <string>
#include "app/EDrive.h"

namespace EDrive {
namespace perception {

class Perception : public EDrive::common::EDriveApp {
 public:
//   Planning() = default;
//   virtual ~Planning();

  std::string Name() const override;

  EDrive::common::Status Init() override;

  EDrive::common::Status Start() override;

  void Stop() override;
};

} // perception
} // EDrive