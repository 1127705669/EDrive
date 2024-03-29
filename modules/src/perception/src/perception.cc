/******************************************************************************
 * Copyright 2022 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#include "perception/src/perception.h"

#include "common/src/adapters/adapter_manager.h"

#include "common/src/util/file.h"

namespace EDrive {
namespace perception {

using EDrive::Result_state;
using EDrive::common::adapter::AdapterManager;

std::string Perception::Name() const { return "EDrive_perception"; }

EDrive::Result_state Perception::Init(){
  ROS_INFO("Perception init, starting...");

  root_path = EDrive::common::util::GetRootPath();
  adapter_conf_file = root_path + adapter_conf_file;
  perception_conf_file = root_path + perception_conf_file;

  ROS_INFO("  registering node: %s", Name().c_str());
  AdapterManager::Init(adapter_conf_file);

  if (!AdapterManager::Initialized()) {
    ROS_INFO("  registering node: %s", Name().c_str());
    AdapterManager::Init(adapter_conf_file);
  }

  return State_Ok;
}

Result_state Perception::CheckInput() {
  AdapterManager::Observe();
  auto objects_adapter = AdapterManager::GetCarlaObjects();
  Carla_objects_ = objects_adapter->GetLatestObserved();
  return State_Ok;
}

EDrive::Result_state Perception::Start(){

  ROS_INFO("Perception resetting vehicle state, sleeping for 1000 ms ...");
  ros::Duration(1.0).sleep();

  timer_ = common::adapter::AdapterManager::CreateTimer(ros::Duration(control_perception_.perception_period()), 
                                                                &Perception::OnTimer,
                                                                this);

  ROS_INFO("Perception init done!");
  ROS_INFO("Perception started");
  
  return State_Ok;
}

void Perception::OnTimer(const ros::TimerEvent &) {
  Result_state state = CheckInput();
  objects_ = Carla_objects_;
  Publish();
}

void Perception::Publish(){
  AdapterManager::PublishPerception(objects_);
}

void Perception::Stop() {
  
}

} // namespace perception
} // namespace EDrive