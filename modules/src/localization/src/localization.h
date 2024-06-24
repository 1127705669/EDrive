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

#include "localization/src/map/vector_map.h"

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <visualization_msgs/MarkerArray.h>

namespace EDrive {
namespace localization {

class Localization : public EDrive::common::EDriveApp {
 public:

  std::string Name() const override;

  common::Result_state Init() override;

  common::Result_state Start() override;

  void Stop() override;

  virtual ~Localization() = default;

 private:

  /* Watch dog timer */
  void OnTimer(const ros::TimerEvent &);

  common::Result_state CheckInput();

  void Publish();

  void PositionConvert();

  void PCDConvert(const std::string& file_path, bool apply_filter, bool save_filtered, const std::string& save_path = "");

  ros::Timer timer_;
  LocalizationConf localization_conf_;

  std::string root_path;
  std::string adapter_conf_file = "/src/localization/conf/adapter.conf";
  std::string localization_conf_file = "/src/localization/conf/localization.conf";
  std::string vector_map_conf_file = "/src/localization/conf/Town05.osm";
  std::string cloud_point_map_conf_file = "/src/localization/conf/Town05.pcd";

  visualization_msgs::Marker position_marker_;
  nav_msgs::Odometry position_odometry_;
  sensor_msgs::PointCloud2 cloud_point_map_;
  visualization_msgs::MarkerArray lanelet2_map_;
  visualization_msgs::MarkerArray vector_map_;
  VectorMap vector_mapper_;
  visualization_msgs::MarkerArray path_;
};

} // namespace localization
} // namespace EDrive