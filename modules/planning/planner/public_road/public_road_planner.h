/******************************************************************************
 * Copyright 2022 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <string>
#include <memory>

#include "modules/planning/planner/planner.h"
#include "modules/common/status/status.h"

namespace EDrive {
namespace planning {

class PublicRoadPlanner : public PlannerWithReferenceLine {
 public:

  PublicRoadPlanner() = delete;

  // explicit PublicRoadPlanner(){}

  virtual ~PublicRoadPlanner() = default;

  void Stop() override {}

  std::string Name() override {return "public_road";}

  common::Status Init() override;

  common::Status Plan() override;

};

}
}