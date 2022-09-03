/******************************************************************************
 * Copyright 2022 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#ifndef EDRIVE_APP_EDRIVE_H_
#define EDRIVE_APP_EDRIVE_H_

#include "ros/ros.h"
#include "modules/common/status/status.h"

namespace EDrive {
namespace common {

class EDriveApp {
 public:

  virtual std::string Name() const = 0;

  // virtual ~EDriveApp() = default;
  // virtual void Stop() = 0;

 private:

 protected:

  virtual EDrive::common::Status Init() = 0;
  virtual EDrive::common::Status Start() = 0;
  virtual void Stop() = 0;

};

} // common
} // EDrive

#define EDRIVE_MAIN(APP)                                       \
  int main(int argc, char *argv[]) {                           \
    APP edrive_app_;                                           \
    ros::init(argc, argv, edrive_app_.Name());                 \
    return 0;                                                  \
  }                                                            \

#endif