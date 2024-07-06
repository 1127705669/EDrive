/******************************************************************************
 * Copyright 2024 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/
#pragma once

#include <string>
#include <vector>

#include <tinyxml2.h>

#include "map/hdmap/adapter/xml_parser/common_define.h"
#include "map/hdmap/adapter/xml_parser/status.h"

namespace EDrive {
namespace hdmap {
namespace adapter {

class SignalsXmlParser {
 public:
  static common::Result_state ParseTrafficLights(
      const tinyxml2::XMLElement& xml_node,
      std::vector<TrafficLightInternal>* traffic_lights);
  static common::Result_state ParseStopSigns(const tinyxml2::XMLElement& xml_node,
                               std::vector<StopSignInternal>* stop_signs);
  static common::Result_state ParseYieldSigns(const tinyxml2::XMLElement& xml_node,
                                std::vector<YieldSignInternal>* yield_signs);

 private:
  static common::Result_state ToPbSignalType(const std::string& xml_type,
                               PbSignalType* signal_type);
  static common::Result_state ToPbSubSignalType(const std::string& xml_type,
                               PbSubSignalType* sub_signal_type);
};

}  // namespace adapter
}  // namespace hdmap
}  // namespace EDrive
