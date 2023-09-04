/******************************************************************************
 * Copyright 2023 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#include "common/src/vehicle_state/vehicle_state_provider.h"

namespace EDrive {
namespace common {

VehicleStateProvider::VehicleStateProvider() {}

double VehicleStateProvider::x() const { return vehicle_state_.x(); }

double VehicleStateProvider::y() const { return vehicle_state_.y(); }

double VehicleStateProvider::z() const { return vehicle_state_.z(); }

double VehicleStateProvider::roll() const { return vehicle_state_.roll(); }

double VehicleStateProvider::pitch() const { return vehicle_state_.pitch(); }

double VehicleStateProvider::yaw() const { return vehicle_state_.yaw(); }

double VehicleStateProvider::heading() const { return vehicle_state_.heading(); }

double VehicleStateProvider::linear_velocity() const { return vehicle_state_.linear_velocity(); }

double VehicleStateProvider::angular_velocity() const { return vehicle_state_.angular_velocity(); }

double VehicleStateProvider::linear_acceleration() const { return vehicle_state_.linear_acceleration(); }

} // namespace common
} // namespace EDrive