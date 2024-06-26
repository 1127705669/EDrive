/******************************************************************************
 * Copyright 2023 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#include "control/common/trajectory_analyzer.h"
#include "common/adapters/adapter_manager.h"

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

}

void TrajectoryAnalyzer::PublishPoint(const ::common::TrajectoryPoint point) const {
  // 创建一个 marker 消息
  visualization_msgs::Marker marker;

  // 设置 marker 的帧 ID 和时间戳
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();

  // 设置 marker 的 namespace 和 id
  marker.ns = "trajectory_point";
  marker.id = 0;

  // 设置 marker 的类型为 SPHERE
  marker.type = visualization_msgs::Marker::SPHERE;

  // 设置 marker 的动作为 ADD
  marker.action = visualization_msgs::Marker::ADD;

  // 设置 marker 的位置
  marker.pose.position.x = point.path_point.x;
  marker.pose.position.y = point.path_point.y;
  marker.pose.position.z = point.path_point.z;

  // 设置 marker 的方向（无旋转）
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  // 设置 marker 的尺寸（直径为 0.2）
  marker.scale.x = 0.8;
  marker.scale.y = 0.8;
  marker.scale.z = 0.8;

  // 设置 marker 的颜色（红色，不透明）
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

} // namespace control
} // namespace EDrive