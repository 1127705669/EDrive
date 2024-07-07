/******************************************************************************
 * Copyright 2022 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/
#include "common/src/EDrive.h"

#include <csignal>
#include <memory>
#include <string>
#include <vector>
#include <iostream>
#include <unistd.h>

#include "gflags/gflags.h"
#include "common/src/log.h"
#include "common/src/state.h"
#include "common/util/file.h"

#include <ros/ros.h>

namespace EDrive {
namespace common {

void EDriveApp::ExportFlags() const {
  const auto root_path = EDrive::common::util::GetRootPath();
  const auto export_file = root_path + "/src/common/data/" + Name() + ".flags";
  // std::cout << export_file << std::endl;
  std::ofstream fout(export_file);
  CHECK(fout) << "Cannot open file " << export_file;

  std::vector<gflags::CommandLineFlagInfo> flags;
  gflags::GetAllFlags(&flags);
  for (const auto& flag : flags) {
    fout << "# " << flag.type << ", default=" << flag.default_value << "\n"
         << "# " << flag.description << "\n"
         << "--" << flag.name << "=" << flag.current_value << "\n"
         << std::endl;
  }
}

int EDrive::common::EDriveApp::Spin() {

  auto status = Init();
  if (State_Ok != status) {
    ROS_INFO(" Init failed: ");
    return -1;
  }

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
  ExportFlags();
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