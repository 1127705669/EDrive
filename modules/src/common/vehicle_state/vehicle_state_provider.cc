/******************************************************************************
 * Copyright 2023 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#include "common/vehicle_state/vehicle_state_provider.h"

#include <cmath>

#include <Eigen/Core>

#include "common/src/log.h"

#include "common/src/log.h"
#include "common/math/euler_angles_zxy.h"
#include "common/math/quaternion.h"

namespace EDrive {
namespace common {

VehicleStateProvider::VehicleStateProvider() {}

Result_state VehicleStateProvider::Update(const nav_msgs::Odometry& localization){
  original_localization_ = localization;
  if (!ConstructExceptLinearVelocity(localization)) {
    EERROR << "Fail to update because ConstructExceptLinearVelocity error.";
    return State_Failed;
  }
  vehicle_state_.set_timestamp(localization.header.stamp.toSec());
  
  vehicle_state_.set_linear_velocity(localization.twist.twist.linear.x);
  vehicle_state_.set_angular_velocity(localization.twist.twist.angular.z);
  return State_Ok;
}

double VehicleStateProvider::QuaternionToHeading(double w, double x, double y, double z) {
    return std::atan2(2.0 * (w * z + x * y), 1 - 2 * (y * y + z * z));
}

bool VehicleStateProvider::ConstructExceptLinearVelocity(const nav_msgs::Odometry& localization) {
  vehicle_state_.set_x(localization.pose.pose.position.x);
  vehicle_state_.set_y(localization.pose.pose.position.y);
  vehicle_state_.set_z(localization.pose.pose.position.z);

  const auto &orientation = localization.pose.pose.orientation;

  vehicle_state_.set_heading(
        QuaternionToHeading(orientation.w, orientation.x,
                            orientation.y, orientation.z));
  
  vehicle_state_.set_angular_velocity(
      localization.twist.twist.angular.z);
  vehicle_state_.set_linear_acceleration(
      localization.twist.twist.linear.y);

  if (!(vehicle_state_.linear_velocity() > 0.0)) {
    vehicle_state_.set_kappa(0.0);
  } else {
    vehicle_state_.set_kappa(vehicle_state_.angular_velocity() /
                             vehicle_state_.linear_velocity());
  }

  math::EulerAnglesZXYd euler_angle(orientation.w, orientation.x,
                                    orientation.y, orientation.z);
  vehicle_state_.set_roll(euler_angle.roll());
  vehicle_state_.set_pitch(euler_angle.pitch());
  vehicle_state_.set_yaw(euler_angle.yaw());

  return true;
}

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

void VehicleStateProvider::set_linear_velocity(const double linear_velocity) {
  vehicle_state_.set_linear_velocity(linear_velocity);
}

math::Vec2d VehicleStateProvider::ComputeCOMPosition(
    const double rear_to_com_distance) const {
  // set length as distance between rear wheel and center of mass.
  Eigen::Vector3d v;
  v << 0.0, rear_to_com_distance, 0.0;
  Eigen::Vector3d pos_vec(vehicle_state_.x(), vehicle_state_.y(),
                          vehicle_state_.z());
  // Initialize the COM position without rotation
  Eigen::Vector3d com_pos_3d = v + pos_vec;
  
  const auto &orientation = original_localization_.pose.pose.orientation;

  Eigen::Quaternion<double> quaternion(orientation.w, orientation.x,
                                        orientation.y, orientation.z);
  // Update the COM position with rotation
  com_pos_3d = quaternion.toRotationMatrix() * v + pos_vec;

  return math::Vec2d(com_pos_3d[0], com_pos_3d[1]);
}

} // namespace common
} // namespace EDrive