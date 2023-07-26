/******************************************************************************
 * Copyright 2022 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <string>
#include "app/EDrive.h"
#include "app/state.h"

namespace EDrive {
namespace control {

class Control : public EDrive::common::EDriveApp {
 public:
//   Planning() = default;
//   virtual ~Planning();

  std::string Name() const override;

  EDrive::Result_state Init() override;

  EDrive::Result_state Start() override;

  void Stop() override;

  virtual ~Control() = default;

 private:

 // Watch dog timer
  void OnTimer(const ros::TimerEvent &);
  
  ros::Timer timer_;
  const float control_period = 0.01;

};

} // control
} // EDrive