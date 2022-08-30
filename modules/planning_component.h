/******************************************************************************
 * Copyright 2022 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

namespace EDrve {
namespace planning {

class PlanningComponent final
    : public EROS::Component<prediction::PredictionObstacles>{

public:
  PlanningComponent() = default;

  ~PlanningComponent() = default;

public:
    bool Init() override;


};
} // planning
} // EDrve
