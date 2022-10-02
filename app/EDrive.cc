/******************************************************************************
 * Copyright 2022 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#include <iostream>
#include <unistd.h>

#include "ros/ros.h"
#include "app/EDrive.h"

namespace EDrive {
namespace common {

int EDrive::common::EDriveApp::Spin() {

  auto status = Init();
  if (!status.ok()) {
    ROS_INFO(" Init failed: ");
    return -1;
  }

  status = Start();
  if (!status.ok()) {
    ROS_INFO(" Start failed: ");
    return -1;
  }

  ros::spin();

  return 0;
}

} // common
} // EDrive