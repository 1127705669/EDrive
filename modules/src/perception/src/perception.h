/******************************************************************************
 * Copyright 2022 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <string>
#include <ros/ros.h>

#include "common/src/EDrive.h"
#include "common/src/state.h"

#include "perception/proto/perception_conf.pb.h"

#include <derived_object_msgs/ObjectArray.h>
#include <sensor_msgs/Image.h>
#include <visualization_msgs/Marker.h>

namespace EDrive {
namespace perception {

class Perception : public EDrive::common::EDriveApp {
 public:
//   Planning() = default;
//   virtual ~Planning();

  std::string Name() const override;

  common::Result_state Init() override;

  common::Result_state Start() override;

  void Stop() override;

 private:
  void OnTimer(const ros::TimerEvent &);
  ros::Timer timer_;
  common::Result_state CheckInput();
  void Publish();
  PerceptionConf control_perception_;

  derived_object_msgs::ObjectArray objects_;
  derived_object_msgs::ObjectArray Carla_objects_;
  sensor_msgs::Image Carla_image_;

  std::string root_path;
  std::string adapter_conf_file = "/src/perception/conf/adapter.conf";
  std::string perception_conf_file = "/src/perception/conf/perception.conf";

 private:
  common::Result_state ConvertImageToVisualization(sensor_msgs::Image& image_msg);
  visualization_msgs::Marker line_strip_;
};

} // perception
} // EDrive