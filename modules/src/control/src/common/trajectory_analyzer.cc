/******************************************************************************
 * Copyright 2023 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#include "control/src/common/trajectory_analyzer.h"

using EDrive::common::PathPoint;
using EDrive::common::TrajectoryPoint;

namespace EDrive {
namespace control {

TrajectoryAnalyzer::TrajectoryAnalyzer(
    const planning::ADCTrajectory *planning_published_trajectory) {

  // for (int i = 0; i < planning_published_trajectory->trajectory_point_size();
  //      ++i) {
  //   trajectory_points_.push_back(
  //       planning_published_trajectory->trajectory_point(i));
  // }
}

// Squared distance from the point to (x, y).
double PointDistanceSquare(const TrajectoryPoint &point, const double x,
                           const double y) {
  const double dx = point.path_point().x() - x;
  const double dy = point.path_point().y() - y;
  return dx * dx + dy * dy;
}

// reference: Optimal trajectory generation for dynamic street scenarios in a
// Frenét Frame,
// Moritz Werling, Julius Ziegler, Sören Kammel and Sebastian Thrun, ICRA 2010
// similar to the method in this paper without the assumption the "normal"
// vector
// (from vehicle position to ref_point position) and reference heading are
// perpendicular.
void TrajectoryAnalyzer::ToTrajectoryFrame(const double x, const double y,
                                           const double theta, const double v,
                                           const PathPoint &ref_point,
                                           double *ptr_s, double *ptr_s_dot,
                                           double *ptr_d,
                                           double *ptr_d_dot) const {

}

PathPoint TrajectoryAnalyzer::QueryMatchedPathPoint(const double x,
                                               const double y) const {
  // double d_min = PointDistanceSquare(trajectory_points_.front(), x, y);
  // size_t index_min = 0;

  // for (size_t i = 1; i < trajectory_points_.size(); ++i) {
  //   double d_temp = PointDistanceSquare(trajectory_points_[i], x, y);
  //   if (d_temp < d_min) {
  //     d_min = d_temp;
  //     index_min = i;
  //   }
  // }
  PathPoint p;
  return p;
}

} // namespace control
} // namespace EDrive