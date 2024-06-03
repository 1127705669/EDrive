/******************************************************************************
 * Copyright 2022 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#include <ros/ros.h>
#include "localization/src/localization.h"

#include "common/src/util/file.h"
#include "common/src/adapters/adapter_manager.h"

namespace EDrive {
namespace localization {

using EDrive::Result_state;
using EDrive::common::adapter::AdapterManager;
  
std::string Localization::Name() const { return "EDrive_localization"; }

EDrive::Result_state Localization::Init(){

  ROS_INFO("Localization init, starting...");

  root_path = EDrive::common::util::GetRootPath();
  adapter_conf_file = root_path + adapter_conf_file;
  localization_conf_file = root_path + localization_conf_file;

  ROS_INFO("  registering node: %s", Name().c_str());
  AdapterManager::Init(adapter_conf_file);

  EDrive::common::util::GetProtoFromASIIFile(localization_conf_file, &localization_conf_);

  return State_Ok;
}

Result_state Localization::CheckInput() {
  AdapterManager::Observe();
  auto position_adapter = AdapterManager::GetVehicle();
  position_odometry_ = position_adapter->GetLatestObserved();

  return State_Ok;
}

EDrive::Result_state Localization::Start(){

  timer_ = common::adapter::AdapterManager::CreateTimer(ros::Duration(localization_conf_.localization_period()), 
                                                                &Localization::OnTimer,
                                                                this);
  ROS_INFO("Localization init done!");
  ROS_INFO("Localization started");
  
  return State_Ok;
}

void Localization::OnTimer(const ros::TimerEvent &) {
  Result_state status = CheckInput();
  PositionConvert();
  Publish();
}

void Localization::PositionConvert() {

    // 设置 frame ID 和时间戳
    position_marker_.header.frame_id = "map";
    position_marker_.header.stamp = ros::Time::now();
    
    // 设置命名空间和ID，这个ID应该在同一个命名空间中是唯一的
    position_marker_.ns = "vehicle_shape";
    position_marker_.id = 0;
    
    // 设置 marker 类型为立方体
    position_marker_.type = visualization_msgs::Marker::CUBE;
    
    // 设置 marker 动作，ADD 为添加，也可用 MODIFY 或 DELETE
    position_marker_.action = visualization_msgs::Marker::ADD;
    
    // 设置位置，从 position_odometry_ 中获取
    position_marker_.pose.position.x = position_odometry_.pose.pose.position.x;
    position_marker_.pose.position.y = position_odometry_.pose.pose.position.y;
    position_marker_.pose.position.z = position_odometry_.pose.pose.position.z;
    
    // 设置姿态
    position_marker_.pose.orientation.x = position_odometry_.pose.pose.orientation.x;
    position_marker_.pose.orientation.y = position_odometry_.pose.pose.orientation.y;
    position_marker_.pose.orientation.z = position_odometry_.pose.pose.orientation.z;
    position_marker_.pose.orientation.w = position_odometry_.pose.pose.orientation.w;
    
    // 设置 marker 的尺寸（假设车的长宽高为2m x 1m x 0.5m）
    position_marker_.scale.x = 2.0;
    position_marker_.scale.y = 1.0;
    position_marker_.scale.z = 0.5;
    
    // 设置颜色和透明度（绿色）
    position_marker_.color.r = 0.0;
    position_marker_.color.g = 1.0;
    position_marker_.color.b = 0.0;
    position_marker_.color.a = 1.0; // 不透明
}

void Localization::Publish(){
  AdapterManager::PublishLocalization(position_odometry_);
  AdapterManager::PublishVehicleLocation(position_marker_);
}

void Localization::Stop() {
  
}

} // namespace localization
} // namespace EDrive