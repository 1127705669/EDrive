/******************************************************************************
 * Copyright 2024 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include "gflags/gflags.h"

// The directory which contains a group of related maps, such as base_map,
// sim_map, routing_topo_grapth, etc.
DECLARE_string(map_dir);
DECLARE_int32(local_utm_zone_id);

DECLARE_string(test_base_map_filename);
DECLARE_string(base_map_filename);
DECLARE_string(sim_map_filename);
DECLARE_string(routing_map_filename);
DECLARE_string(end_way_point_filename);
DECLARE_string(speed_control_filename);

DECLARE_string(vehicle_config_path);

DECLARE_bool(use_ros_time);

DECLARE_string(localization_tf2_frame_id);
DECLARE_string(localization_tf2_child_frame_id);
DECLARE_bool(use_navigation_mode);
DECLARE_string(navigation_mode_end_way_point_file);
