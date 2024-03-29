/******************************************************************************
 * Copyright 2023 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <vector>
#include <ros/ros.h>

#include "common/src/vehicle_state/vehicle_state_provider.h"

#include "planning/ADCTrajectory.h"

namespace EDrive {
namespace control {

/**
 * @class TrajectoryAnalyzer
 * @brief process point query and conversion related to trajectory
 */
class TrajectoryAnalyzer {
 public:
  /**
   * @brief constructor
   */
  TrajectoryAnalyzer() = default;

  /**
   * @brief constructor
   * @param planning_published_trajectory trajectory data generated by
   * planning module
   */
  TrajectoryAnalyzer(
      const planning::ADCTrajectory *planning_published_trajectory);

  /**
   * @brief destructor
   */
  ~TrajectoryAnalyzer() = default;

  /**
   * @brief query a point on trajectery that its position is closest
   * to the given position.
   * @param x value of x-coordination in the given position
   * @param y value of y-coordination in the given position
   * @return a point on trajectory, the point may be a point of trajectory
   * or interpolated by two adjacent points of trajectory
   */
  ::common::PathPoint QueryMatchedPathPoint(const double x, const double y) const;

  /**
   * @brief query a point of trajectery that its position is closest
   * to the given position.
   * @param x value of x-coordination in the given position
   * @param y value of y-coordination in the given position
   * @return a point of trajectory
   */
  ::common::TrajectoryPoint QueryNearestPointByPosition(const double x,
                                                      const double y) const;

  /**
   * @brief convert a position with theta and speed to trajectory frame,
   * - longitudinal and lateral direction to the trajectory
   * @param x x-value of the position
   * @param y y-value of the position
   * @param theta heading angle on the position
   * @param v speed on the position
   * @param matched_point matched point on trajectory for the given position
   * @param ptr_s longitudinal distance
   * @param ptr_s_dot longitudinal speed
   * @param ptr_d lateral distance
   * @param ptr_d_dot lateral speed
   */
  void ToTrajectoryFrame(const double x, const double y, const double theta,
                         const double v, const ::common::PathPoint &matched_point,
                         double *ptr_s, double *ptr_s_dot, double *ptr_d,
                         double *ptr_d_dot) const;

 private:
  std::vector<::common::TrajectoryPoint> trajectory_points_;
};

} // namespace control
} // namespace EDrive