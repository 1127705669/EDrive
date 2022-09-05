/******************************************************************************
 * Copyright 2022 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include "modules/planning/planner/planner.h"

namespace EDrive {
namespace planning {

class Lattice_planner : public Planner
{
 private:
    
 public:
  Lattice_planner(/* args */);
  ~Lattice_planner();
};

} // planning
} // EDrive