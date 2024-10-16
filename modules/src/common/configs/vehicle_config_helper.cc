/******************************************************************************
 * Copyright 2017 The EDrive Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "common/configs/vehicle_config_helper.h"

#include <algorithm>
#include <cmath>

#include "common/util/file.h"

namespace EDrive {
namespace common {

// initalize static variables
std::string VehicleConfigHelper::root_path = "";
std::string VehicleConfigHelper::conf_file = "";

VehicleConfig VehicleConfigHelper::vehicle_config_;
bool VehicleConfigHelper::is_init_ = false;

VehicleConfigHelper::VehicleConfigHelper() {}

void VehicleConfigHelper::Init() {
  root_path = EDrive::common::util::GetRootPath();
  conf_file = "/src/common/data/model3_config.pb.txt";
  root_path += conf_file;
  Init(root_path); 
}

void VehicleConfigHelper::Init(const std::string &config_file) {
  VehicleConfig params;
  EDrive::common::util::GetProtoFromASCIIFile(config_file, &params);
  Init(params);
}

void VehicleConfigHelper::Init(const VehicleConfig &vehicle_params) {
  vehicle_config_ = vehicle_params;
  is_init_ = true;
}

const VehicleConfig &VehicleConfigHelper::GetConfig() {
  if (!is_init_) {
    Init();
  }
  return vehicle_config_;
}

double VehicleConfigHelper::MinSafeTurnRadius() {
  const auto &param = vehicle_config_.vehicle_param();
  double lat_edge_to_center =
      std::max(param.left_edge_to_center(), param.right_edge_to_center());
  double lon_edge_to_center =
      std::max(param.front_edge_to_center(), param.back_edge_to_center());
  return std::sqrt((lat_edge_to_center + param.min_turn_radius()) *
                       (lat_edge_to_center + param.min_turn_radius()) +
                   lon_edge_to_center * lon_edge_to_center);
}

}  // namespace common
}  // namespace EDrive
