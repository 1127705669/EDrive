/******************************************************************************
 * Copyright 2022 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#ifndef EDRIVE_APP_EDRIVE_H_
#define EDRIVE_APP_EDRIVE_H_

#include <csignal>
#include <string>

#include "gflags/gflags.h"
#include "common/src/log.h"
#include "common/src/state.h"

#include <ros/ros.h>

namespace EDrive {
namespace common {

class EDriveApp {
 public:

  virtual std::string Name() const = 0;

  virtual int Spin();

  // virtual ~EDriveApp() = default;
  
  uint32_t callback_thread_num_ = 1;

 private:
  /**
   * @brief Export flag values to <FLAGS_log_dir>/<name>.flags.
   */
  void ExportFlags() const;

 protected:

  virtual common::Result_state Init() = 0;
  virtual common::Result_state Start() = 0;
  virtual void Stop() = 0;

};

} // common
} // EDrive

#define EDRIVE_MAIN(APP)                                       \
  int main(int argc, char *argv[]) {                           \
    google::InitGoogleLogging(argv[0]);                        \
    google::ParseCommandLineFlags(&argc, &argv, true);         \
    APP edrive_app_;                                           \
    ros::init(argc, argv, edrive_app_.Name());                 \
    edrive_app_.Spin();                                        \
    return 0;                                                  \
  }                                                            \

#endif