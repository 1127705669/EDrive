/******************************************************************************
 * Copyright 2022 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <memory>
#include <string>

#include "modules/common/status/status.h"

namespace EDrive {
namespace planning {

class Planner {
 public:
  Planner() = delete;

//  explicit Planner(const ) :

  virtual ~Planner() = default;
 
  virtual EDrive::common::Status Init() = 0;
 
  virtual EDrive::common::Status Plan() = 0;
 
 protected:

};

class PlannerWithReferenceLine : public Planner {
 public:
  PlannerWithReferenceLine() = delete;
 
  virtual ~PlannerWithReferenceLine() = default;

};

} // planning
} // EDrive
