/******************************************************************************
 * Copyright 2024 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

/**
 * @file
 **/

#include "common/src/math/cartesian_frenet_conversion.h"

#include <cmath>

namespace EDrive {
namespace common {
namespace math {

void CartesianFrenetConverter::cartesian_to_frenet(
    const double rs, const double rx, const double ry, const double rtheta,
    const double rkappa, const double rdkappa, const double x, const double y,
    const double v, const double a, const double theta, const double kappa,
    std::array<double, 3>* const ptr_s_condition,
    std::array<double, 3>* const ptr_d_condition) {
  const double dx = x - rx;
  const double dy = y - ry;

  const double cos_theta_r = std::cos(rtheta);
  const double sin_theta_r = std::sin(rtheta);

  const double cross_rd_nd = cos_theta_r * dy - sin_theta_r * dx;
  ptr_d_condition->at(0) =
      std::copysign(std::sqrt(dx * dx + dy * dy), cross_rd_nd);

  const double delta_theta = theta - rtheta;
  const double tan_delta_theta = std::tan(delta_theta);
  const double cos_delta_theta = std::cos(delta_theta);

  const double one_minus_kappa_r_d = 1 - rkappa * ptr_d_condition->at(0);
  ptr_d_condition->at(1) = one_minus_kappa_r_d * tan_delta_theta;

  const double kappa_r_d_prime =
      rdkappa * ptr_d_condition->at(0) + rkappa * ptr_d_condition->at(1);

  ptr_d_condition->at(2) =
      -kappa_r_d_prime * tan_delta_theta +
      one_minus_kappa_r_d / cos_delta_theta / cos_delta_theta *
          (kappa * one_minus_kappa_r_d / cos_delta_theta - rkappa);

  ptr_s_condition->at(0) = rs;

  ptr_s_condition->at(1) = v * cos_delta_theta / one_minus_kappa_r_d;

  const double delta_theta_prime =
      one_minus_kappa_r_d / cos_delta_theta * kappa - rkappa;
  ptr_s_condition->at(2) =
      (a * cos_delta_theta -
       ptr_s_condition->at(1) * ptr_s_condition->at(1) *
           (ptr_d_condition->at(1) * delta_theta_prime - kappa_r_d_prime)) /
      one_minus_kappa_r_d;
  return;
}

void CartesianFrenetConverter::cartesian_to_frenet(
    const double rs, const double rx, const double ry, const double rtheta,
    const double x, const double y, double* ptr_s, double* ptr_d) {
  const double dx = x - rx;
  const double dy = y - ry;

  const double cos_theta_r = std::cos(rtheta);
  const double sin_theta_r = std::sin(rtheta);

  const double cross_rd_nd = cos_theta_r * dy - sin_theta_r * dx;
  *ptr_d = std::copysign(std::sqrt(dx * dx + dy * dy), cross_rd_nd);
  *ptr_s = rs;
  return;
}

double CartesianFrenetConverter::CalculateKappa(const double rkappa,
                                                const double rdkappa,
                                                const double l, const double dl,
                                                const double ddl) {
  double denominator = (dl * dl + (1 - l * rkappa) * (1 - l * rkappa));
  if (std::fabs(denominator) < 1e-8) {
    return 0.0;
  }
  denominator = std::pow(denominator, 1.5);
  const double numerator = rkappa + ddl - 2 * l * rkappa * rkappa -
                           l * ddl * rkappa + l * l * rkappa * rkappa * rkappa +
                           l * dl * rdkappa + 2 * dl * dl * rkappa;
  return numerator / denominator;
}

double CartesianFrenetConverter::CalculateLateralDerivative(
    const double rtheta, const double theta, const double l,
    const double rkappa) {
  return (1 - rkappa * l) * std::tan(theta - rtheta);
}

double CartesianFrenetConverter::CalculateSecondOrderLateralDerivative(
    const double rtheta, const double theta, const double rkappa,
    const double kappa, const double rdkappa, const double l) {
  const double dl = CalculateLateralDerivative(rtheta, theta, l, rkappa);
  const double theta_diff = theta - rtheta;
  const double cos_theta_diff = std::cos(theta_diff);
  const double res = -(rdkappa * l + rkappa * dl) * std::tan(theta - rtheta) +
                     (1 - rkappa * l) / (cos_theta_diff * cos_theta_diff) *
                         (kappa * (1 - rkappa * l) / cos_theta_diff - rkappa);
  // if (std::isinf(res)) {
  //   AWARN << "result is inf when calculate second order lateral "
  //            "derivative. input values are rtheta:"
  //         << rtheta << " theta: " << theta << ", rkappa: " << rkappa
  //         << ", kappa: " << kappa << ", rdkappa: " << rdkappa << ", l: " << l
  //         << std::endl;
  // }
  return res;
}

}  // namespace math
}  // namespace common
}  // namespace EDrive