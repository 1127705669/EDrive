/******************************************************************************
 * Copyright 2022 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <string>
#include "common/src/EDrive.h"
#include "common/src/state.h"

#include "common/adapters/adapter_manager.h"

#include "planning/ADCTrajectory.h"

namespace EDrive {
namespace planning {

using EDrive::common::adapter::AdapterManager;

class Planning : public EDrive::common::EDriveApp {
 public:
  Planning() = default;
  virtual ~Planning();

  std::string Name() const override;

  common::Result_state Init() override;

  common::Result_state Start() override;

  void Stop() override;

  /**
   * @brief main logic of the planning module, runs periodically triggered by
   * timer.
   */
  void RunOnce();

  /**
   * @brief Fill the header and publish the planning message.
   */
  void Publish(::planning::ADCTrajectory* trajectory) {
    
    AdapterManager::PublishPlanning(*trajectory);
  }

 private:
  // Watch dog timer
  void OnTimer(const ros::TimerEvent &);

  common::Result_state RegisterPlanners();

  void CheckInput();

  void PublishPlanningPb(::planning::ADCTrajectory* trajectory_pb);
  const double planning_period = 0.1;
  ros::Timer timer_;

  nav_msgs::Odometry position_;
  std::string root_path;
  std::string adapter_conf_file = "/src/planning/conf/adapter.conf";
  std::string planning_conf_file = "/src/planning/conf/planning.conf";

  /* test code */
  double start_point_x;
  double start_point_y;
  bool is_initialized = false;
};

} // namespace planning
} // namespace EDrive