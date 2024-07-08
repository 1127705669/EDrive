/******************************************************************************
 * Copyright 2024 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/
#pragma once

#include <string>
#include <vector>

#include "tinyxml2.h"

#include "map/hdmap/adapter/xml_parser/common_define.h"
#include "map/hdmap/adapter/xml_parser/status.h"

namespace EDrive {
namespace hdmap {
namespace adapter {

class LanesXmlParser {
 public:
  static common::Result_state Parse(const tinyxml2::XMLElement& xml_node,
                      const std::string& road_id,
                      std::vector<RoadSectionInternal>* sections);

 private:
  static common::Result_state ParseLaneSection(const tinyxml2::XMLElement& xml_node,
                                 std::vector<LaneInternal>* lanes);

  static common::Result_state ParseSectionBoundary(const tinyxml2::XMLElement& xml_node,
                                     PbBoundaryPolygon* boundary);

  static common::Result_state ToPbBoundaryType(const std::string& type,
                                 PbBoundaryEdgeType* boundary_type);
  static common::Result_state ParseLane(const tinyxml2::XMLElement& xml_node,
                          LaneInternal* lane_internal);
  static common::Result_state ParseDirection(const tinyxml2::XMLElement& xml_node,
                               PbLane* lane);
  static common::Result_state ParseCenterCurve(const tinyxml2::XMLElement& xml_node,
                                 PbLane* lane);
  static common::Result_state ParseSpeed(const tinyxml2::XMLElement& xml_node, PbLane* lane);
  static common::Result_state ParseSampleAssociates(const tinyxml2::XMLElement& xml_node,
                                      PbLane* lane);
  static common::Result_state ParseRoadSampleAssociates(const tinyxml2::XMLElement& xml_node,
                                      PbLane* lane);
  static common::Result_state ParseObjectOverlapGroup(
      const tinyxml2::XMLElement& xml_node,
      std::vector<OverlapWithLane>* object_overlaps);
  static common::Result_state ParseSignalOverlapGroup(
      const tinyxml2::XMLElement& xml_node,
      std::vector<OverlapWithLane>* signal_overlaps);
  static common::Result_state ParseJunctionOverlapGroup(
      const tinyxml2::XMLElement& xml_node,
      std::vector<OverlapWithLane>* junction_overlaps);
  static common::Result_state ParseLaneOverlapGroup(
      const tinyxml2::XMLElement& xml_node,
      std::vector<OverlapWithLane>* lane_overlaps);

  static common::Result_state ToPbLaneType(const std::string& type, PbLaneType* pb_type);
  static common::Result_state ToPbTurnType(const std::string& type, PbTurnType* pb_turn_type);
  static common::Result_state ToPbDirection(const std::string& type,
                              PbLaneDirection* pb_direction);

  static void ParseLaneLink(const tinyxml2::XMLElement& xml_node,
                            PbLane* lane);
  static common::Result_state ParseLaneBorderMark(const tinyxml2::XMLElement& xml_node,
                                    PbLaneBoundaryTypeType* boundary_type);
  static common::Result_state ToPbLaneMarkType(const std::string& type,
                                 const std::string& color,
                                 PbLaneBoundaryTypeType* boundary_type);
};

}  // namespace adapter
}  // namespace hdmap
}  // namespace EDrive
