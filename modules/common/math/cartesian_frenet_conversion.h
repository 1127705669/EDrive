/******************************************************************************
 * Copyright 2022 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

namespace EDrive {
namespace common {
namespace math {

class CartesianFrenetConverter {
 public:
  CartesianFrenetConverter() = delete;

  static void cartesian_to_frenet();

  static void frenet_to_cartesian();

  static double CalculateTheta();

  static double CalculateKappa();

};

} // math
} // common
} // EDrive