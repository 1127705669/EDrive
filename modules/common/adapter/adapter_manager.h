/******************************************************************************
 * Copyright 2022 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "modules/common/adapters/adapter.h"

#include "ros/include/ros/ros.h"

/**
 * @namespace EDrive::common::adapter
 * @brief EDrive::common::adapter
 */
namespace EDrive {
namespace common {
namespace adapter {

class AdapterManager {
 public:
  static void Init(const std::string &adapter_config_filename);
};

} // adapter
} // common
} // EDrive