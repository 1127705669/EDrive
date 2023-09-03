/******************************************************************************
 * Copyright 2023 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#include "control/src/common/trajectory_analyzer.h"

namespace EDrive {
namespace control {

TrajectoryAnalyzer::TrajectoryAnalyzer(
    const planning::ADCTrajectory *planning_published_trajectory) {
}

// Squared distance from the point to (x, y).
double PointDistanceSquare(const double x,
                           const double y) {
  
  return 0;
}

void TrajectoryAnalyzer::QueryMatchedPathPoint(const double x, 
                                               const double y) const {
}

} // namespace control
} // namespace EDrive