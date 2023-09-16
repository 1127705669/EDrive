/******************************************************************************
 * Copyright 2022 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

/**
 * @file viewer.h
 *
 * @brief Declaration of the class Viewer.
 */
#pragma once

#include <string>
#include "common/src/EDrive.h"
#include "common/src/state.h"

#include "viewer/src/common/viewer_agent.h"

#include "viewer/proto/viewer_conf.pb.h"

#include "planning/ADCTrajectory.h"

namespace EDrive {
namespace viewer {

class Viewer : public EDrive::common::EDriveApp {
 public:

  std::string Name() const override;

  EDrive::Result_state Init() override;

  EDrive::Result_state Start() override;

  void Stop() override;

 private:
  // Watch dog timer
  void OnTimer(const ros::TimerEvent &);

  EDrive::Result_state CheckInput();

  ViewerAgent viewer_agent_;

  ViewerConf viewer_conf_;

  ros::Timer timer_;
  const double viewer_period = 0.01;
  planning::ADCTrajectory trajectory_;

  std::string root_path;
  std::string adapter_conf_file = "/src/viewer/conf/adapter.conf";
  std::string viewer_conf_file = "/src/viewer/conf/viewer.conf";
};

} // namespace viewer
} // namespace EDrive