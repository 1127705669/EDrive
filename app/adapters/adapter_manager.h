/******************************************************************************
 * Copyright 2023 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <functional>
#include <memory>
#include <string>
#include <type_traits>
#include <vector>

#include "ros/ros.h"

namespace EDrive {
namespace common {
namespace adapter{

class AdapterManager
{
 public:

    /**
   * @brief create a timer which will call a callback at the specified
   * rate. It takes a class member function, and a bare pointer to the
   * object to call the method on.
   */

  template <class T>
  static ros::Timer CreateTimer(ros::Duration period,
                                void (T::*callback)(const ros::TimerEvent &),
                                T *obj,ros::NodeHandle &nh, bool oneshot = false,
                                bool autostart = true) {
    return nh.createTimer(period, callback, obj, oneshot, autostart);
  }
 private:

};

} // adapter
} // common
} // EDrive