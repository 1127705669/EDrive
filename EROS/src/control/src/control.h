/******************************************************************************
 * Copyright 2022 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <string>

#include "app/EDrive.h"
#include "app/state.h"

#include "EROS/src/control/src/controller/controller_agent.h"

namespace EDrive {
namespace control {

using EDrive::Result_state;

class Control : public EDrive::common::EDriveApp {
 public:

  std::string Name() const override;

  Result_state Init() override;

  Result_state Start() override;

  void Stop() override;

  virtual ~Control() = default;

 private:

 // Watch dog timer
  void OnTimer(const ros::TimerEvent &);

  void SendCmd();

  Result_state ProduceControlCommand();
  
  ros::Timer timer_;
  const float control_period = 0.01;
  ros::Time init_time_;
  ControllerAgent controller_agent_;
  
};

} // namespace control
} // namespace EDrive