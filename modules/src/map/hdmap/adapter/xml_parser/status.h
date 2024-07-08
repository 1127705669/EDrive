/******************************************************************************
 * Copyright 2024 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/
#pragma once

#include <cstddef>
#include <ostream>
#include <sstream>
#include <string>
#include <vector>
#include <iostream>

#include "common/src/state.h"

namespace EDrive {
namespace hdmap {
namespace adapter {

using EDrive::common::Result_state;

#define RETURN_IF_ERROR(expr)                            \
  do {                                                   \
    const EDrive::common::Result_state status_ = (expr); \
    if (!Result_state::State_Ok) return status_;         \
  } while (0)

}  // namespace adapter
}  // namespace hdmap
}  // namespace EDrive
