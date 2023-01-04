/******************************************************************************
 * Copyright 2022 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#include <iostream>
#include <unistd.h>

#include "ros/ros.h"
#include "app/EDrive.h"
#include "app/state.h"

namespace EDrive {
namespace common {

int EDrive::common::EDriveApp::Spin() {

  auto status = Init();
  if (State_Ok != status) {
    ROS_INFO(" Init failed: ");
    return -1;
  }

  status = Start();
  if (State_Ok != status) {
    ROS_INFO(" Start failed: ");
    return -1;
  }

  ros::spin();

  ros::waitForShutdown();
  Stop();

  ROS_INFO(" exited. ");

  return 0;
}

} // common
} // EDrive