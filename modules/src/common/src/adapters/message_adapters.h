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
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

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
using VehicleLocationAdapter = Adapter<visualization_msgs::Marker>;
using CarlaObjectsAdapter = Adapter<derived_object_msgs::ObjectArray>;
using ViewerObjectsAdapter = Adapter<visualization_msgs::MarkerArray>;
using PerceptionAdapter = Adapter<derived_object_msgs::ObjectArray>;
using LocalizationAdapter = Adapter<nav_msgs::Odometry>;
using ViewerPathAdapter = Adapter<nav_msgs::Path>;
using RoadMarkingsAdapter = Adapter<sensor_msgs::Image>;
using RoadMarkerAdapter = Adapter<visualization_msgs::Marker>;
using VectorMapAdapter = Adapter<visualization_msgs::MarkerArray>;
using CloudPointMapAdapter = Adapter<sensor_msgs::PointCloud2>;

} // namespace adapter
} // namespace common
} // namespace EDrive