/******************************************************************************
 * Copyright 2024 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#include "common/src/log.h"

#include "gtest/gtest.h"

#include "glog/logging.h"

namespace EDrive {
namespace common {

TEST(LogTest, TestAll) { EINFO << "11111"; }

}  // namespace common
}  // namespace EDrive
