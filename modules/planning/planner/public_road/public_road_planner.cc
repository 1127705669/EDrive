/******************************************************************************
 * Copyright 2022 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#include "modules/planning/planner/planner.h"
#include "modules/planning/planner/public_road/public_road_planner.h"


namespace EDrive {
namespace planning {

class PublicRoadPlanner : public PlannerWithReferenceLine {
 public:

    PublicRoadPlanner() = delete;

    explicit PublicRoadPlanner();

    virtual ~PublicRoadPlanner() = default;

    void Stop() override {}

    common::Status Init(const PlanningConfig& config) override;

    common::Status Plan(const common::TrajectoryPoint& planning_init_point,
                      Frame* frame,
                      ADCTrajectory* ptr_computed_trajectory) override;

};

} // planning
} // EDrive