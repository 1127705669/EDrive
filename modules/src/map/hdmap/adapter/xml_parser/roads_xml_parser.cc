/******************************************************************************
 * Copyright 2024 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/
#include <string>
#include <vector>

#include "map/hdmap/adapter/xml_parser/lanes_xml_parser.h"
#include "map/hdmap/adapter/xml_parser/objects_xml_parser.h"
#include "map/hdmap/adapter/xml_parser/roads_xml_parser.h"
#include "map/hdmap/adapter/xml_parser/signals_xml_parser.h"
#include "map/hdmap/adapter/xml_parser/util_xml_parser.h"

namespace {
bool IsRoadBelongToJunction(const std::string& road_id) {
  CHECK(!road_id.empty());
  return road_id != "-1";
}
}  // namespace

namespace EDrive {
namespace hdmap {
namespace adapter {

Result_state RoadsXmlParser::Parse(const tinyxml2::XMLElement& xml_node,
                             std::vector<RoadInternal>* roads) {
  CHECK_NOTNULL(roads);

  auto road_node = xml_node.FirstChildElement("road");
  while (road_node) {
    // road attributes
    std::string id;
    std::string junction_id;
    int checker = UtilXmlParser::QueryStringAttribute(*road_node, "id", &id);
    checker += UtilXmlParser::QueryStringAttribute(*road_node, "junction",
                                                   &junction_id);
    if (checker != tinyxml2::XML_SUCCESS) {
      std::string err_msg = "Error parsing road attributes";
      return Result_state::State_Failed;
    }
    RoadInternal road_internal;
    road_internal.id = id;
    road_internal.road.mutable_id()->set_id(id);
    if (IsRoadBelongToJunction(junction_id)) {
      road_internal.road.mutable_junction_id()->set_id(junction_id);
    }
    // lanes
    RETURN_IF_ERROR(LanesXmlParser::Parse(*road_node, road_internal.id,
                                          &road_internal.sections));

    // objects
    auto sub_node = road_node->FirstChildElement("objects");
    if (sub_node != nullptr) {
      // stop line
      ObjectsXmlParser::ParseStopLines(*sub_node, &road_internal.stop_lines);
      // crosswalks
      ObjectsXmlParser::ParseCrosswalks(*sub_node, &road_internal.crosswalks);
      // clearareas
      ObjectsXmlParser::ParseClearAreas(*sub_node, &road_internal.clear_areas);
      // speed_bumps
      ObjectsXmlParser::ParseSpeedBumps(*sub_node, &road_internal.speed_bumps);
      // parking_spaces
      ObjectsXmlParser::ParseParkingSpaces(
                                    *sub_node, &road_internal.parking_spaces);
    }

    // signals
    sub_node = road_node->FirstChildElement("signals");
    if (sub_node != nullptr) {
      // traffic lights
      SignalsXmlParser::ParseTrafficLights(*sub_node,
                                           &road_internal.traffic_lights);
      // stop signs
      SignalsXmlParser::ParseStopSigns(*sub_node, &road_internal.stop_signs);
      // yield signs
      SignalsXmlParser::ParseYieldSigns(*sub_node, &road_internal.yield_signs);
    }

    roads->push_back(road_internal);
    road_node = road_node->NextSiblingElement("road");
  }

  return Result_state::State_Ok;
}

}  // namespace adapter
}  // namespace hdmap
}  // namespace EDrive
