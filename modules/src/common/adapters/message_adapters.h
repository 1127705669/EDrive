/******************************************************************************
 * Copyright 2023 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <derived_object_msgs/ObjectArray.h>
#include <nav_msgs/Odometry.h>
// #include "control/proto/control_cmd.pb.h"
#include "common/adapters/adapter.h"
#include "control/ControlCommand.h"
#include "control/CarlaEgoVehicleControl.h"
#include "planning/ADCTrajectory.h"
#include "viewer/VisualizingData.h"
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include "common/CarlaEgoVehicleStatus.h"
#include "map/relative_map/proto/navigation.pb.h"
#include "routing/proto/routing.pb.h"

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
using ViewerLocalizationAdapter = Adapter<visualization_msgs::Marker>;
using ViewerPathAdapter = Adapter<nav_msgs::Path>;
using RoadMarkingsAdapter = Adapter<sensor_msgs::Image>;
using RoadMarkerAdapter = Adapter<visualization_msgs::Marker>;
using VectorMapAdapter = Adapter<visualization_msgs::MarkerArray>;
using CloudPointMapAdapter = Adapter<sensor_msgs::PointCloud2>;
using FixedPathAdapter = Adapter<visualization_msgs::MarkerArray>;
using VehicleStatusAdapter = Adapter<::common::CarlaEgoVehicleStatus>;
using MarkerDebugPointAdapter = Adapter<visualization_msgs::Marker>;
using NavigationAdapter = Adapter<relative_map::NavigationInfo>;
using RelativeMapAdapter = Adapter<relative_map::MapMsg>;
using RoutingRequestAdapter = Adapter<routing::RoutingRequest>;
using RoutingResponseAdapter = Adapter<routing::RoutingResponse>;

} // namespace adapter
} // namespace common
} // namespace EDrive