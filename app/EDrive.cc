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
  ros::spin();

  return 0;
}

} // common
} // EDrive