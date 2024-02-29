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

  root_path = EDrive::common::util::GetRootPath();
  adapter_conf_file = root_path + adapter_conf_file;
  localization_conf_file = root_path + localization_conf_file;

  ROS_INFO("  registering node: %s", Name().c_str());
  AdapterManager::Init(adapter_conf_file);

  return State_Ok;
}

Result_state Localization::CheckInput() {
  AdapterManager::Observe();
  auto position_adapter = AdapterManager::GetVehicle();
  position_ = position_adapter->GetLatestObserved();
  return State_Ok;
}

EDrive::Result_state Localization::Start(){

  timer_ = common::adapter::AdapterManager::CreateTimer(ros::Duration(localization_conf_.localization_period()), 
                                                                &Localization::OnTimer,
                                                                this);
  ROS_INFO("Control init done!");
  ROS_INFO("Control started");
  
  return State_Ok;
}

void Localization::OnTimer(const ros::TimerEvent &) {

}

void Localization::Stop() {
  
}

} // namespace localization
} // namespace EDrive