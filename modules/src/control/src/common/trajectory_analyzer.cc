/******************************************************************************
 * Copyright 2023 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#include "common/trajectory_analyzer.h"

namespace EDrive {
namespace control {

// Squared distance from the point to (x, y).
double PointDistanceSquare(const control_msg::TrajectoryPoint &point, const double x,
                           const double y) {
  const double dx = point.path_point.x - x;
  const double dy = point.path_point.y - y;
  return dx * dx + dy * dy;
}

control_msg::PathPoint TrajectoryAnalyzer::QueryMatchedPathPoint(const double x, 
                                                                 const double y) const {
  control_msg::PathPoint point;
  
  double d_min = PointDistanceSquare(trajectory_points_.front(), x, y);
  size_t index_min = 0;
  
  for (size_t i = 1; i < trajectory_points_.size(); ++i) {
    double d_temp = PointDistanceSquare(trajectory_points_[i], x, y);
    if (d_temp < d_min) {
      d_min = d_temp;
      index_min = i;
    }
  }
  return point;
}

} // namespace control
} // namespace EDrive