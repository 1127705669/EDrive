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
  ROS_INFO("init ok");

  std::unique_ptr<ros::AsyncSpinner> spinner;
  if (callback_thread_num_ > 1) {
    spinner = std::unique_ptr<ros::AsyncSpinner>(
        new ros::AsyncSpinner(callback_thread_num_));
  }

  status = Start();
  if (State_Ok != status) {
    ROS_INFO(" Start failed: ");
    return -2;
  }

  ROS_INFO("start ok");

  if (spinner) {
    spinner->start();
  } else {
    ros::spin();
  }

  ros::waitForShutdown();
  Stop();

  ROS_INFO(" exited. ");

  return 0;
}

} // common
} // EDrive