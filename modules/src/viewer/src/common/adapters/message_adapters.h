/******************************************************************************
 * Copyright 2023 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <nav_msgs/Odometry.h>
#include "viewer/src/common/adapters/adapter.h"
#include "control/ControlCommand.h"
#include "control/CarlaEgoVehicleControl.h"
#include "planning/ADCTrajectory.h"
#include "viewer/VisualizingData.h"
#include "derived_object_msgs/ObjectArray.h"
#include "visualization_msgs/MarkerArray.h"

/**
 * @file message_adapters.h
 * @namespace EDrive::common::adapter
 * @brief This is an agglomeration of all the message adapters supported that
 * specializes the adapter template.
 */
namespace EDrive {
namespace viewer {
namespace adapter {

using ControlCommandAdapter = Adapter<::control::CarlaEgoVehicleControl>;
using PlanningAdapter = Adapter<::planning::ADCTrajectory>;
using ViewerAdapter = Adapter<::viewer::VisualizingData>;

using CARLAVehicleAdapter = Adapter<nav_msgs::Odometry>;
using CARLAObjectsAdapter = Adapter<derived_object_msgs::ObjectArray>;

using ViewerObjectsAdapter = Adapter<visualization_msgs::MarkerArray>;
using ViewerVehicleAdapter = Adapter<visualization_msgs::Marker>;

} // namespace adapter
} // namespace viewer
} // namespace EDrive