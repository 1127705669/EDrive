/******************************************************************************
 * Copyright 2022 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <string>
#include "modules/common/data/error_code.h"

namespace EDrive {
namespace common {

class Status {
 public:
  
  Status() : code_(ErrorCode::OK), msg_() {}
  // explicit Status() {}
  // ~Status() = default;

  static Status OK() { return Status(); }

  bool ok() const { return code_ == ErrorCode::OK; }

 private:
  ErrorCode code_;
  std::string msg_;
};

} // common
} // EDrive