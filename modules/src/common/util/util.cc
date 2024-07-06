/******************************************************************************
 * Copyright 2024 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#include "common/util/util.h"

#include <cmath>
#include <vector>

namespace EDrive {
namespace common {
namespace util {

SLPoint MakeSLPoint(const double s, const double l) {
  SLPoint sl;
  sl.set_s(s);
  sl.set_l(l);
  return sl;
}

PointENU MakePointENU(const double x, const double y, const double z) {
  PointENU point_enu;
  point_enu.set_x(x);
  point_enu.set_y(y);
  point_enu.set_z(z);
  return point_enu;
}

PointENU operator+(const PointENU enu, const math::Vec2d& xy) {
  PointENU point;
  point.set_x(enu.x() + xy.x());
  point.set_y(enu.y() + xy.y());
  point.set_z(enu.z());
  return point;
}

PointENU MakePointENU(const math::Vec2d& xy) {
  PointENU point_enu;
  point_enu.set_x(xy.x());
  point_enu.set_y(xy.y());
  point_enu.set_z(0.0);
  return point_enu;
}

EDrive::perception::Point MakePerceptionPoint(const double x, const double y,
                                              const double z) {
  perception::Point point3d;
  point3d.set_x(x);
  point3d.set_y(y);
  point3d.set_z(z);
  return point3d;
}

SpeedPoint MakeSpeedPoint(const double s, const double t, const double v,
                          const double a, const double da) {
  SpeedPoint speed_point;
  speed_point.set_s(s);
  speed_point.set_t(t);
  speed_point.set_v(v);
  speed_point.set_a(a);
  speed_point.set_da(da);
  return speed_point;
}

PathPoint MakePathPoint(const double x, const double y, const double z,
                        const double theta, const double kappa,
                        const double dkappa, const double ddkappa) {
  PathPoint path_point;
  path_point.set_x(x);
  path_point.set_y(y);
  path_point.set_z(z);
  path_point.set_theta(theta);
  path_point.set_kappa(kappa);
  path_point.set_dkappa(dkappa);
  path_point.set_ddkappa(ddkappa);
  return path_point;
}

PathPoint GetWeightedAverageOfTwoPathPoints(const PathPoint& p1,
                                            const PathPoint& p2,
                                            const double w1, const double w2) {
  PathPoint p;
  p.set_x(p1.x() * w1 + p2.x() * w2);
  p.set_y(p1.y() * w1 + p2.y() * w2);
  p.set_z(p1.z() * w1 + p2.z() * w2);
  p.set_theta(p1.theta() * w1 + p2.theta() * w2);
  p.set_kappa(p1.kappa() * w1 + p2.kappa() * w2);
  p.set_dkappa(p1.dkappa() * w1 + p2.dkappa() * w2);
  p.set_ddkappa(p1.ddkappa() * w1 + p2.ddkappa() * w2);
  p.set_s(p1.s() * w1 + p2.s() * w2);
  return p;
}

}  // namespace util
}  // namespace common
}  // namespace EDrive
