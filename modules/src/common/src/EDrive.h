/******************************************************************************
 * Copyright 2022 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#ifndef EDRIVE_APP_EDRIVE_H_
#define EDRIVE_APP_EDRIVE_H_

#include <ros/ros.h>

#include "common/src/state.h"

namespace EDrive {
namespace common {

class EDriveApp {
 public:

  virtual std::string Name() const = 0;

  virtual int Spin();

  // virtual ~EDriveApp() = default;
  // virtual void Stop() = 0;

  uint32_t callback_thread_num_ = 1;

 private:

 protected:

  virtual EDrive::Result_state Init() = 0;
  virtual EDrive::Result_state Start() = 0;
  virtual void Stop() = 0;

};

} // common
} // EDrive

#define EDRIVE_MAIN(APP)                                       \
  int main(int argc, char *argv[]) {                           \
    APP edrive_app_;                                           \
    ros::init(argc, argv, edrive_app_.Name());                 \
    edrive_app_.Spin();                                        \
    return 0;                                                  \
  }                                                            \

#endif