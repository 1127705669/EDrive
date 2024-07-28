/******************************************************************************
 * Copyright 2023 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#include "control/common/trajectory_analyzer.h"
#include "common/adapters/adapter_manager.h"

#include <algorithm>
#include <cmath>
#include <utility>

#include "Eigen/Core"

#include "common/src/log.h"

namespace math = EDrive::common::math;
using ::common::PathPoint;
using ::common::TrajectoryPoint;
using EDrive::common::adapter::AdapterManager;

namespace EDrive {
namespace control {

TrajectoryAnalyzer::TrajectoryAnalyzer(const planning::ADCTrajectory *planning_published_trajectory) {
  header_time_ = planning_published_trajectory->header.stamp.toSec();
  seq_num_ = planning_published_trajectory->header.seq;
  for (int i = 0; i < planning_published_trajectory->trajectory_point.size();
       ++i) {
    trajectory_points_.push_back(
        planning_published_trajectory->trajectory_point[i]);
  }
}

// Squared distance from the point to (x, y).
double PointDistanceSquare(const TrajectoryPoint &point, const double x,
                           const double y) {
  const double dx = point.path_point.x - x;
  const double dy = point.path_point.y - y;
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
  double dx = x - ref_point.x;
  double dy = y - ref_point.y;

  double cos_ref_theta = std::cos(ref_point.theta);
  double sin_ref_theta = std::sin(ref_point.theta);

  // the sin of diff angle between vector (cos_ref_theta, sin_ref_theta) and
  // (dx, dy)
  double cross_rd_nd = cos_ref_theta * dy - sin_ref_theta * dx;
  *ptr_d = cross_rd_nd;

  // the cos of diff angle between vector (cos_ref_theta, sin_ref_theta) and
  // (dx, dy)
  double dot_rd_nd = dx * cos_ref_theta + dy * sin_ref_theta;
  *ptr_s = ref_point.s + dot_rd_nd;

  double delta_theta = theta - ref_point.theta;
  double cos_delta_theta = std::cos(delta_theta);
  double sin_delta_theta = std::sin(delta_theta);

  *ptr_d_dot = v * sin_delta_theta;

  double one_minus_kappa_r_d = 1 - ref_point.kappa * (*ptr_d);

  if (one_minus_kappa_r_d <= 0.0) {
    EERROR << "TrajectoryAnalyzer::ToTrajectoryFrame "
              "found fatal reference and actual difference. "
              "Control output might be unstable:"
           << " ref_point.kappa:" << ref_point.kappa
           << " ref_point.x:" << ref_point.x
           << " ref_point.y:" << ref_point.y << " car x:" << x
           << " car y:" << y << " *ptr_d:" << *ptr_d
           << " one_minus_kappa_r_d:" << one_minus_kappa_r_d;
    // currently set to a small value to avoid control crash.
    one_minus_kappa_r_d = 0.01;
  }

  *ptr_s_dot = v * cos_delta_theta / one_minus_kappa_r_d;
}

void TrajectoryAnalyzer::PublishPoint(const ::common::TrajectoryPoint point) const {
  // Create a marker message
  visualization_msgs::Marker marker;

  // Set the marker's frame ID and timestamp
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();

  // Set the marker's namespace and ID
  marker.ns = "trajectory_point";
  marker.id = 0;

  // Set the marker type to SPHERE
  marker.type = visualization_msgs::Marker::SPHERE;

  // Set the marker action to ADD
  marker.action = visualization_msgs::Marker::ADD;

  // Set the marker's position
  marker.pose.position.x = point.path_point.x;
  marker.pose.position.y = point.path_point.y;
  marker.pose.position.z = point.path_point.z;

  // Set the marker's orientation (no rotation)
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  // Set the marker's scale (diameter 0.8)
  marker.scale.x = 0.8;
  marker.scale.y = 0.8;
  marker.scale.z = 0.8;

// Set the marker's color (red, fully opaque)
  marker.color.r = 1.0f;
  marker.color.g = 0.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;
  
  AdapterManager::PublishMarkerDebugPoint(marker);
}

PathPoint TrajectoryAnalyzer::QueryMatchedPathPoint(const double x,
                                               const double y) const {
  double d_min = PointDistanceSquare(trajectory_points_.front(), x, y);
  size_t index_min = 0;

  for (size_t i = 1; i < trajectory_points_.size(); ++i) {
    double d_temp = PointDistanceSquare(trajectory_points_[i], x, y);
    if (d_temp < d_min) {
      d_min = d_temp;
      index_min = i;
    }
  }

  return trajectory_points_[index_min].path_point;
}

TrajectoryPoint TrajectoryAnalyzer::QueryNearestPointByAbsoluteTime(
    const double t) const {
  return QueryNearestPointByRelativeTime(t - header_time_);
}

TrajectoryPoint TrajectoryAnalyzer::QueryNearestPointByRelativeTime(
    const double t) const {
  auto func_comp = [](const TrajectoryPoint &point,
                      const double relative_time) {
    return point.relative_time < relative_time;
  };

  auto it_low = std::lower_bound(trajectory_points_.begin(),
                                 trajectory_points_.end(), t, func_comp);

  if (it_low == trajectory_points_.begin()) {
    return trajectory_points_.front();
  }

  if (it_low == trajectory_points_.end()) {
    return trajectory_points_.back();
  }

  auto it_lower = it_low - 1;
  if (it_low->relative_time - t < t - it_lower->relative_time) {
    return *it_low;
  }
  return *it_lower;
}

TrajectoryPoint TrajectoryAnalyzer::QueryNearestPointByPosition(
    const double x, const double y) const {
  double d_min = PointDistanceSquare(trajectory_points_.front(), x, y);
  size_t index_min = 0;

  for (size_t i = 1; i < trajectory_points_.size(); ++i) {
    double d_temp = PointDistanceSquare(trajectory_points_[i], x, y);
    if (d_temp < d_min) {
      d_min = d_temp;
      index_min = i;
    }
  }
  const TrajectoryPoint point = trajectory_points_[index_min];
  PublishPoint(point);
  return trajectory_points_[index_min];
}

void TrajectoryAnalyzer::TrajectoryTransformToCOM(
    const double rear_to_com_distance) {
  CHECK_GT(trajectory_points_.size(), 0U);
  for (size_t i = 0; i < trajectory_points_.size(); ++i) {
    auto com = ComputeCOMPosition(rear_to_com_distance,
                                  trajectory_points_[i].path_point);
    trajectory_points_[i].path_point.x = com.x();
    trajectory_points_[i].path_point.y = com.y();
  }
}

common::math::Vec2d TrajectoryAnalyzer::ComputeCOMPosition(
    const double rear_to_com_distance, const ::common::PathPoint &path_point) const {
  // Initialize the vector for coordinate transformation of the position
  // reference point
  Eigen::Vector3d v;
  const double cos_heading = std::cos(path_point.theta);
  const double sin_heading = std::sin(path_point.theta);
  v << rear_to_com_distance * cos_heading, rear_to_com_distance * sin_heading,
      0.0;
  // Original position reference point at center of rear-axis
  Eigen::Vector3d pos_vec(path_point.x, path_point.y, path_point.z);
  // Transform original position with vector v
  Eigen::Vector3d com_pos_3d = v + pos_vec;
  // Return transfromed x and y
  return common::math::Vec2d(com_pos_3d[0], com_pos_3d[1]);
}

} // namespace control
} // namespace EDrive