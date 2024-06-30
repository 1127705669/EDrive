/******************************************************************************
 * Copyright 2024 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include "common/vehicle_state/vehicle_state_provider.h"

namespace EDrive {
namespace control {

class DependencyInjector {
 public:
  DependencyInjector() = default;

  ~DependencyInjector() = default;

  EDrive::common::VehicleStateProvider* vehicle_state() {
    return &vehicle_state_;
  }

 private:
  EDrive::common::VehicleStateProvider vehicle_state_;
};

}  // namespace control
}  // namespace EDrive
