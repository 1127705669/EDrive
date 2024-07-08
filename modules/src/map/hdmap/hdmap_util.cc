/******************************************************************************
 * Copyright 2024 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/
#include "map/hdmap/hdmap_util.h"

#include "common/adapters/adapter_manager.h"
#include "common/util/file.h"
#include "common/util/string_tokenizer.h"

namespace EDrive {
namespace hdmap {

using EDrive::common::adapter::AdapterManager;
using EDrive::relative_map::MapMsg;

namespace {

// Find the first existing file from a list of candidates: "file_a|file_b|...".
std::string FindFirstExist(const std::string& dir, const std::string& files) {
  const auto candidates =
      EDrive::common::util::StringTokenizer::Split(files, "|");
  for (const auto& filename : candidates) {
    const std::string file_path =
        EDrive::common::util::StrCat(FLAGS_map_dir, "/", filename);
    if (EDrive::common::util::PathExists(file_path)) {
      return file_path;
    }
  }
  AERROR << "No existing file found in " << dir << "/" << files
         << ". Fallback to first candidate as default result.";
  CHECK(!candidates.empty()) << "Please specify at least one map.";
  return EDrive::common::util::StrCat(FLAGS_map_dir, "/", candidates[0]);
}

}  // namespace

const SpeedControls* GetSpeedControls() {
  static std::unique_ptr<SpeedControls> speed_control_;
  if (speed_control_ == nullptr) {
    speed_control_.reset(new SpeedControls());
    if (!common::util::GetProtoFromFile(SpeedControlFile(),
                                        speed_control_.get())) {
      speed_control_->Clear();
    }
  }
  return speed_control_.get();
}  // namespace hdmap

std::string BaseMapFile() {
  if (FLAGS_use_navigation_mode) {
    AWARN << "base_map file is not used when FLAGS_use_navigation_mode is true";
  }
  return FLAGS_test_base_map_filename.empty()
             ? FindFirstExist(FLAGS_map_dir, FLAGS_base_map_filename)
             : FindFirstExist(FLAGS_map_dir, FLAGS_test_base_map_filename);
}

std::string SimMapFile() {
  if (FLAGS_use_navigation_mode) {
    AWARN << "sim_map file is not used when FLAGS_use_navigation_mode is true";
  }
  return FindFirstExist(FLAGS_map_dir, FLAGS_sim_map_filename);
}

std::string RoutingMapFile() {
  if (FLAGS_use_navigation_mode) {
    AWARN << "routing_map file is not used when FLAGS_use_navigation_mode is "
             "true";
  }
  return FindFirstExist(FLAGS_map_dir, FLAGS_routing_map_filename);
}

std::unique_ptr<HDMap> CreateMap(const std::string& map_file_path) {
  std::unique_ptr<HDMap> hdmap(new HDMap());
  if (hdmap->LoadMapFromFile(map_file_path) != 0) {
    AERROR << "Failed to load HDMap " << map_file_path;
    return nullptr;
  }
  AINFO << "Load HDMap success: " << map_file_path;
  return hdmap;
}

std::unique_ptr<HDMap> CreateMap(const MapMsg& map_msg) {
  std::unique_ptr<HDMap> hdmap(new HDMap());
  if (hdmap->LoadMapFromProto(map_msg.hdmap()) != 0) {
    AERROR << "Failed to load RelativeMap: "
           << map_msg.header().ShortDebugString();
    return nullptr;
  }
  return hdmap;
}

std::unique_ptr<HDMap> HDMapUtil::base_map_ = nullptr;
uint64_t HDMapUtil::base_map_seq_ = 0;
std::mutex HDMapUtil::base_map_mutex_;

std::unique_ptr<HDMap> HDMapUtil::sim_map_ = nullptr;
std::mutex HDMapUtil::sim_map_mutex_;

const HDMap* HDMapUtil::BaseMapPtr() {
  if (FLAGS_use_navigation_mode) {
    std::lock_guard<std::mutex> lock(base_map_mutex_);
    auto* relative_map = AdapterManager::GetRelativeMap();
    if (!relative_map) {
      AERROR << "RelativeMap adapter is not registered";
      return nullptr;
    }
    if (relative_map->Empty()) {
      AERROR << "RelativeMap is empty";
      return nullptr;
    }
    const auto& latest = relative_map->GetLatestObserved();
    if (base_map_ != nullptr &&
        base_map_seq_ == latest.header().sequence_num()) {
      // avoid re-create map in the same cycle.
      return base_map_.get();
    } else {
      base_map_ = CreateMap(latest);
      base_map_seq_ = latest.header().sequence_num();
    }
  } else if (base_map_ == nullptr) {
    std::lock_guard<std::mutex> lock(base_map_mutex_);
    if (base_map_ == nullptr) {  // Double check.
      base_map_ = CreateMap(BaseMapFile());
    }
  }
  return base_map_.get();
}

const HDMap& HDMapUtil::BaseMap() { return *CHECK_NOTNULL(BaseMapPtr()); }

const HDMap* HDMapUtil::SimMapPtr() {
  if (FLAGS_use_navigation_mode) {
    return BaseMapPtr();
  } else if (sim_map_ == nullptr) {
    std::lock_guard<std::mutex> lock(sim_map_mutex_);
    if (sim_map_ == nullptr) {  // Double check.
      sim_map_ = CreateMap(SimMapFile());
    }
  }
  return sim_map_.get();
}

const HDMap& HDMapUtil::SimMap() { return *CHECK_NOTNULL(SimMapPtr()); }

bool HDMapUtil::ReloadMaps() {
  {
    std::lock_guard<std::mutex> lock(base_map_mutex_);
    base_map_ = CreateMap(BaseMapFile());
  }
  {
    std::lock_guard<std::mutex> lock(sim_map_mutex_);
    sim_map_ = CreateMap(SimMapFile());
  }
  return base_map_ != nullptr && sim_map_ != nullptr;
}

}  // namespace hdmap
}  // namespace EDrive
