/******************************************************************************
 * Copyright 2023 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/
/**
 * @file vehicle_state.h
 *
 * @brief Declaration of the class VehicleStateProvider.
 */
#pragma once

#include <memory>
#include <string>

#include "common/src/macro.h"
#include "common/src/vehicle_state/proto/vehicle_state.pb.h"

namespace EDrive {
namespace common {

class VehicleStateProvider {
 public:
    /**
   * @brief Default destructor.
   */
  virtual ~VehicleStateProvider() = default;

  /**
   * @brief Get the x-coordinate of vehicle position.
   * @return The x-coordinate of vehicle position.
   */
  double x() const;

  /**
   * @brief Get the y-coordinate of vehicle position.
   * @return The y-coordinate of vehicle position.
   */
  double y() const;

  /**
   * @brief Get the z coordinate of vehicle position.
   * @return The z coordinate of vehicle position.
   */
  double z() const;

  double kappa() const;

  /**
   * @brief Get the vehicle roll angle.
   * @return The euler roll angle.
   */
  double roll() const;

  /**
   * @brief Get the vehicle pitch angle.
   * @return The euler pitch angle.
   */
  double pitch() const;

  /**
   * @brief Get the vehicle yaw angle.
   *  As of now, use the heading instead of yaw angle.
   *  Heading angle with East as zero, yaw angle has North as zero
   * @return The euler yaw angle.
   */
  double yaw() const;

  /**
   * @brief Get the heading of vehicle position, which is the angle
   *        between the vehicle's heading direction and the x-axis.
   * @return The angle between the vehicle's heading direction
   *         and the x-axis.
   */
  double heading() const;

  /**
   * @brief Get the vehicle's linear velocity.
   * @return The vehicle's linear velocity.
   */
  double linear_velocity() const;

  /**
   * @brief Get the vehicle's angular velocity.
   * @return The vehicle's angular velocity.
   */
  double angular_velocity() const;

  /**
   * @brief Get the vehicle's linear acceleration.
   * @return The vehicle's linear acceleration.
   */
  double linear_acceleration() const;

 private:
  common::VehicleState vehicle_state_;
  DECLARE_SINGLETON(VehicleStateProvider);
};

} // namespace common
} // namespace EDrive