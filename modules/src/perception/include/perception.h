/******************************************************************************
 * Copyright 2022 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <string>
#include <ros/ros.h>

#include "common/src/EDrive.h"
#include "common/src/state.h"

#include <derived_object_msgs/ObjectArray.h>

namespace EDrive {
namespace perception {

class Perception : public EDrive::common::EDriveApp {
 public:
//   Planning() = default;
//   virtual ~Planning();

  std::string Name() const override;

  EDrive::Result_state Init() override;

  EDrive::Result_state Start() override;

  void Stop() override;

 private:
  void OnTimer(const ros::TimerEvent &);
  ros::Timer timer_;
  EDrive::Result_state CheckInput();
  void Publish();

  derived_object_msgs::ObjectArray objects_;

  std::string root_path;
  std::string adapter_conf_file = "/src/perception/conf/adapter.conf";
  std::string perception_conf_file = "/src/perception/conf/perception.conf";
};

} // perception
} // EDrive