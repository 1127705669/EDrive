/******************************************************************************
 * Copyright 2022 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <string>
#include "common/src/EDrive.h"
#include "common/src/state.h"

namespace EDrive {
namespace routing {

class Routing : public EDrive::common::EDriveApp {
 public:

  std::string Name() const override;

  common::Result_state Init() override;

  common::Result_state Start() override;

  void Stop() override;
};

} // namespace routing
} // namespace EDrive