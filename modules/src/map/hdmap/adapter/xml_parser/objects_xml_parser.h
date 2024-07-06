/******************************************************************************
 * Copyright 2024 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/
#pragma once

#include <vector>

#include <tinyxml2.h>

#include "map/hdmap/adapter/xml_parser/common_define.h"
#include "map/hdmap/adapter/xml_parser/status.h"

namespace EDrive {
namespace hdmap {
namespace adapter {

class ObjectsXmlParser {
 public:
  static common::Result_state ParseCrosswalks(const tinyxml2::XMLElement& xml_node,
                                std::vector<PbCrosswalk>* crosswalks);
  static common::Result_state ParseClearAreas(const tinyxml2::XMLElement& xml_node,
                                std::vector<PbClearArea>* clear_areas);
  static common::Result_state ParseSpeedBumps(const tinyxml2::XMLElement& xml_node,
                                std::vector<PbSpeedBump>* speed_bumps);
  static common::Result_state ParseStopLines(const tinyxml2::XMLElement& xml_node,
                               std::vector<StopLineInternal>* stop_lines);
  static common::Result_state ParseParkingSpaces(const tinyxml2::XMLElement& xml_node,
                            std::vector<PbParkingSpace>* parking_spaces);
};

}  // namespace adapter
}  // namespace hdmap
}  // namespace EDrive
