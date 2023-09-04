/******************************************************************************
 * Copyright 2022 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <string>
#include "common/src/EDrive.h"
#include "common/src/state.h"
#include "planning/ADCTrajectory.h"

namespace EDrive {
namespace planning {

class Planning : public EDrive::common::EDriveApp {
 public:
//   Planning() = default;
//   virtual ~Planning();

  std::string Name() const override;

  EDrive::Result_state Init() override;

  EDrive::Result_state Start() override;

  void Stop() override;

 private:
  // Watch dog timer
  void OnTimer(const ros::TimerEvent &);
  const double planning_period = 0.1;
  ros::Timer timer_;
};

} // planning
} // EDrive