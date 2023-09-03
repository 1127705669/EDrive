/******************************************************************************
 * Copyright 2022 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/
/**
 * @file viewer.h
 *
 * @brief Declaration of the class Viewer.
 */
#pragma once

#include <string>
#include "common/src/EDrive.h"
#include "common/src/state.h"

namespace EDrive {
namespace viewer {

class Viewer : public EDrive::common::EDriveApp {
 public:

  std::string Name() const override;

  EDrive::Result_state Init() override;

  EDrive::Result_state Start() override;

  void Stop() override;
};

} // namespace viewer
} // namespace EDrive