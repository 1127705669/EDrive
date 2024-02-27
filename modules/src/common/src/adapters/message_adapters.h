/******************************************************************************
 * Copyright 2023 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <derived_object_msgs/ObjectArray.h>
#include <nav_msgs/Odometry.h>
// #include "control/proto/control_cmd.pb.h"
#include "common/src/adapters/adapter.h"
#include "control/ControlCommand.h"
#include "control/CarlaEgoVehicleControl.h"
#include "planning/ADCTrajectory.h"
#include "viewer/VisualizingData.h"

/**
 * @file message_adapters.h
 * @namespace EDrive::common::adapter
 * @brief This is an agglomeration of all the message adapters supported that
 * specializes the adapter template.
 */
namespace EDrive {
namespace common {
namespace adapter {

using ControlCommandAdapter = Adapter<::control::CarlaEgoVehicleControl>;
using PlanningAdapter = Adapter<::planning::ADCTrajectory>;
using ViewerAdapter = Adapter<::viewer::VisualizingData>;
using VehicleAdapter = Adapter<nav_msgs::Odometry>;
using CarlaObjectsAdapter = Adapter<derived_object_msgs::ObjectArray>;

} // namespace adapter
} // namespace common
} // namespace EDrive