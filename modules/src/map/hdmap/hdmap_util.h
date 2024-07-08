/******************************************************************************
 * Copyright 2024 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <memory>
#include <mutex>
#include <string>

#include "map/proto/map_id.pb.h"
#include "map/proto/map_speed_control.pb.h"

#include "common/configs/config_gflags.h"
#include "common/util/file.h"
#include "common/util/string_util.h"
#include "map/hdmap/hdmap.h"

/**
 * @namespace EDrive::hdmap
 * @brief EDrive::hdmap
 */
namespace EDrive {
namespace hdmap {

/**
 * @brief get base map file path from flags.
 * @return base map path
 */
std::string BaseMapFile();

/**
 * @brief get simulation map file path from flags.
 * @return simulation map path
 */
std::string SimMapFile();

/**
 * @brief get routing map file path from flags.
 * @return routing map path
 */
std::string RoutingMapFile();

/**
 * @brief get end way point file path from flags.
 * @return end way point file path
 */
inline std::string EndWayPointFile() {
  if (FLAGS_use_navigation_mode) {
    return EDrive::common::util::StrCat(
        FLAGS_navigation_mode_end_way_point_file);
  } else {
    return EDrive::common::util::StrCat(FLAGS_map_dir, "/",
                                        FLAGS_end_way_point_filename);
  }
}

inline std::string SpeedControlFile() {
  return EDrive::common::util::StrCat(FLAGS_map_dir, "/",
                                      FLAGS_speed_control_filename);
}

const SpeedControls* GetSpeedControls();

/**
 * @brief create a Map ID given a string.
 * @param id a string id
 * @return a Map ID instance
 */
inline EDrive::hdmap::Id MakeMapId(const std::string& id) {
  EDrive::hdmap::Id map_id;
  map_id.set_id(id);
  return map_id;
}

std::unique_ptr<HDMap> CreateMap(const std::string& map_file_path);

class HDMapUtil {
 public:
  // Get default base map from the file specified by global flags.
  // Return nullptr if failed to load.
  static const HDMap* BaseMapPtr();
  // Guarantee to return a valid base_map, or else raise fatal error.
  static const HDMap& BaseMap();

  // Get default sim_map from the file specified by global flags.
  // Return nullptr if failed to load.
  static const HDMap* SimMapPtr();

  // Guarantee to return a valid sim_map, or else raise fatal error.
  static const HDMap& SimMap();

  // Reload maps from the file specified by global flags.
  static bool ReloadMaps();

 private:
  HDMapUtil() = delete;

  static std::unique_ptr<HDMap> base_map_;
  static uint64_t base_map_seq_;
  static std::mutex base_map_mutex_;

  static std::unique_ptr<HDMap> sim_map_;
  static std::mutex sim_map_mutex_;
};

}  // namespace hdmap
}  // namespace EDrive
