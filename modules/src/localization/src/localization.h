/******************************************************************************
 * Copyright 2022 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <string>
#include <ros/ros.h>

#include "common/src/EDrive.h"
#include "common/src/state.h"

#include "localization/proto/localization_conf.pb.h"

#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>

namespace EDrive {
namespace localization {

using EDrive::Result_state;

class Localization : public EDrive::common::EDriveApp {
 public:

  std::string Name() const override;

  EDrive::Result_state Init() override;

  EDrive::Result_state Start() override;

  void Stop() override;

  virtual ~Localization() = default;

 private:

  /* Watch dog timer */
  void OnTimer(const ros::TimerEvent &);

  EDrive::Result_state CheckInput();

  void Publish();

  void PositionConvert();

  ros::Timer timer_;
  LocalizationConf localization_conf_;

  std::string root_path;
  std::string adapter_conf_file = "/src/localization/conf/adapter.conf";
  std::string localization_conf_file = "/src/localization/conf/localization.conf";

  visualization_msgs::Marker position_marker_;
  nav_msgs::Odometry position_odometry_;

};

} // namespace localization
} // namespace EDrive