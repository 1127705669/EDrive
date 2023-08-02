/******************************************************************************
 * Copyright 2023 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include "control/proto/control_cmd.pb.h"
#include "common/src/adapters/adapter.h"

/**
 * @file message_adapters.h
 * @namespace EDrive::common::adapter
 * @brief This is an agglomeration of all the message adapters supported that
 * specializes the adapter template.
 */
namespace EDrive {
namespace common {
namespace adapter {

using ControlCommandAdapter = Adapter<control::ControlCommand>;

} // namespace adapter
} // namespace common
} // namespace EDrive