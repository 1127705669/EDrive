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

    explicit Status(ErrorCode Code = ErrorCode::OK, std::string_view )


};

} // common
} // EDrive