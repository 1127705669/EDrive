/******************************************************************************
 * Copyright 2024 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

/**
 * @file reference_line.h
 **/

#pragma once

#include <string>
#include <utility>
#include <vector>

#include "common/proto/pnc_point.pb.h"
#include "map/proto/map_geometry.pb.h"
#include "planning/proto/sl_boundary.pb.h"
#include "routing/proto/routing.pb.h"

#include "common/math/vec2d.h"
#include "map/pnc_map/path.h"
#include "planning/reference_line/reference_point.h"

namespace EDrive {
namespace planning {

class ReferenceLine {
 public:
  ReferenceLine() = default;
  explicit ReferenceLine(const ReferenceLine& reference_line) = default;
  template <typename Iterator>
  explicit ReferenceLine(const Iterator begin, const Iterator end)
      : reference_points_(begin, end),
        map_path_(std::move(std::vector<hdmap::MapPathPoint>(begin, end))) {}
  explicit ReferenceLine(const std::vector<ReferencePoint>& reference_points);
  explicit ReferenceLine(const hdmap::Path& hdmap_path);

  /** Stitch current reference line with the other reference line
   * The stitching strategy is to use current reference points as much as
   * possible. The following two examples show two successful stitch cases.
   *
   * Example 1
   * this:   |--------A-----x-----B------|
   * other:                 |-----C------x--------D-------|
   * Result: |------A-----x-----B------x--------D-------|
   * In the above example, A-B is current reference line, and C-D is the other
   * reference line. If part B and part C matches, we update current reference
   * line to A-B-D.
   *
   * Example 2
   * this:                  |-----A------x--------B-------|
   * other:  |--------C-----x-----D------|
   * Result: |--------C-----x-----A------x--------B-------|
   * In the above example, A-B is current reference line, and C-D is the other
   * reference line. If part A and part D matches, we update current reference
   * line to C-A-B.
   *
   * @return false if these two reference line cannot be stitched
   */
  bool Stitch(const ReferenceLine& other);

  bool Shrink(const common::math::Vec2d& point, double look_backward,
              double look_forward);

  const hdmap::Path& map_path() const;
  const std::vector<ReferencePoint>& reference_points() const;

  ReferencePoint GetReferencePoint(const double s) const;
  std::vector<ReferencePoint> GetReferencePoints(double start_s,
                                                 double end_s) const;

  std::size_t GetNearestReferenceIndex(const double s) const;

  ReferencePoint GetNearestReferencePoint(const common::math::Vec2d& xy) const;

  std::vector<hdmap::LaneSegment> GetLaneSegments(const double start_s,
                                                  const double end_s) const;

  ReferencePoint GetNearestReferencePoint(const double s) const;

  ReferencePoint GetReferencePoint(const double x, const double y) const;

  bool GetApproximateSLBoundary(const common::math::Box2d& box,
                                const double start_s, const double end_s,
                                SLBoundary* const sl_boundary) const;
  bool GetSLBoundary(const common::math::Box2d& box,
                     SLBoundary* const sl_boundary) const;
  bool GetSLBoundary(const hdmap::Polygon& polygon,
                     SLBoundary* const sl_boundary) const;

  bool SLToXY(const common::SLPoint& sl_point,
              common::math::Vec2d* const xy_point) const;
  bool XYToSL(const common::math::Vec2d& xy_point,
              common::SLPoint* const sl_point) const;
  template <class XYPoint>
  bool XYToSL(const XYPoint& xy, common::SLPoint* const sl_point) const {
    return XYToSL(common::math::Vec2d(xy.x(), xy.y()), sl_point);
  }

  bool GetLaneWidth(const double s, double* const lane_left_width,
                    double* const lane_right_width) const;

  bool GetRoadWidth(const double s, double* const road_left_width,
                    double* const road_right_width) const;

  void GetLaneFromS(const double s,
                    std::vector<hdmap::LaneInfoConstPtr>* lanes) const;

  bool IsOnRoad(const common::SLPoint& sl_point) const;
  bool IsOnRoad(const common::math::Vec2d& vec2d_point) const;
  template <class XYPoint>
  bool IsOnRoad(const XYPoint& xy) const {
    return IsOnRoad(common::math::Vec2d(xy.x(), xy.y()));
  }
  bool IsOnRoad(const SLBoundary& sl_boundary) const;

  /**
   * @brief Check if a box is blocking the road surface. The crieria is to check
   * whether the remaining space on the road surface is larger than the provided
   * gap space.
   * @param boxed the provided box
   * @param gap check the gap of the space
   * @return true if the box blocks the road.
   */
  bool IsBlockRoad(const common::math::Box2d& box2d, double gap) const;

  /**
   * @brief check if any part of the box has overlap with the road.
   */
  bool HasOverlap(const common::math::Box2d& box) const;

  double Length() const { return map_path_.length(); }

  std::string DebugString() const;

  double GetSpeedLimitFromS(const double s) const;

  void AddSpeedLimit(const hdmap::SpeedControl& speed_control);
  void AddSpeedLimit(double start_s, double end_s, double speed_limit);

 private:
  /**
   * @brief Linearly interpolate p0 and p1 by s0 and s1.
   * The input has to satisfy condition: s0 <= s <= s1
   * p0 and p1 must have lane_waypoint.
   * Note: it requires p0 and p1 are on the same lane, adjacent lanes, or
   * parallel neighboring lanes. Otherwise the interpolated result may not
   * valid.
   * @param p0 the first anchor point for interpolation.
   * @param s0 the longitutial distance (s) of p0 on current reference line.
   * s0 <= s && s0 <= s1
   * @param p1 the second anchor point for interpolation
   * @param s1 the longitutial distance (s) of p1 on current reference line.
   * s1
   * @param s identifies the the middle point that is going to be
   * interpolated.
   * s >= s0 && s <= s1
   * @return The interpolated ReferencePoint.
   */
  static ReferencePoint Interpolate(const ReferencePoint& p0, const double s0,
                                    const ReferencePoint& p1, const double s1,
                                    const double s);
  ReferencePoint InterpolateWithMatchedIndex(
      const ReferencePoint& p0, const double s0, const ReferencePoint& p1,
      const double s1, const hdmap::InterpolatedIndex& index) const;

  static double FindMinDistancePoint(const ReferencePoint& p0, const double s0,
                                     const ReferencePoint& p1, const double s1,
                                     const double x, const double y);

 private:
  struct SpeedLimit {
    double start_s = 0.0;
    double end_s = 0.0;
    double speed_limit = 0.0;  // unit m/s
    SpeedLimit() = default;
    SpeedLimit(double _start_s, double _end_s, double _speed_limit)
        : start_s(_start_s), end_s(_end_s), speed_limit(_speed_limit) {}
  };
  /**
   * This speed limit overrides the lane speed limit
   **/
  std::vector<SpeedLimit> speed_limit_;
  std::vector<ReferencePoint> reference_points_;
  hdmap::Path map_path_;
};

}  // namespace planning
}  // namespace EDrive

#endif  // MODULES_PLANNING_REFERENCE_LINE_REFERENCE_LINE_H_
