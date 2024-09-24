/******************************************************************************
 * Copyright 2024 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#include "common/configs/config_gflags.h"

DEFINE_string(map_dir, "/home/ethan/workset/EDrive/modules/src/map/data",
              "Directory which contains a group of related maps.");
DEFINE_int32(local_utm_zone_id, 10, "UTM zone id.");

DEFINE_string(test_base_map_filename, "",
              "If not empty, use this test base map files.");

DEFINE_string(base_map_filename, "base_map.bin|base_map.xml|base_map.txt",
              "Base map files in the map_dir, search in order.");
DEFINE_string(sim_map_filename, "sim_map.bin|sim_map.txt",
              "Simulation map files in the map_dir, search in order.");
DEFINE_string(routing_map_filename, "routing_map.bin|routing_map.txt",
              "Routing map files in the map_dir, search in order.");
DEFINE_string(end_way_point_filename, "default_end_way_point.txt",
              "End way point of the map, will be sent in RoutingRequest.");
DEFINE_string(speed_control_filename, "speed_control.pb.txt",
              "The speed control region in a map.");

DEFINE_string(vehicle_config_path, "common/data/mkz_config.pb.txt",
              "the file path of vehicle config file");

DEFINE_bool(use_ros_time, false,
            "Whether Clock::Now() gets time from system_clock::now() or from "
            "ros::Time::now().");

DEFINE_string(localization_tf2_frame_id, "world", "the tf2 transform frame id");
DEFINE_string(localization_tf2_child_frame_id, "localization",
              "the tf2 transform child frame id");

DEFINE_bool(use_navigation_mode, false,
            "Use relative position in navigation mode");
DEFINE_string(
    navigation_mode_end_way_point_file,
    "dreamview/conf/navigation_mode_default_end_way_point.txt",
    "end_way_point file used if navigation mode is set.");
