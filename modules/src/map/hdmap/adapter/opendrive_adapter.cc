/******************************************************************************
 * Copyright 2024 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/
#include "map/hdmap/adapter/opendrive_adapter.h"

#include <vector>

#include "common/src/log.h"
#include "map/hdmap/adapter/proto_organizer.h"
#include "map/hdmap/adapter/xml_parser/status.h"

namespace EDrive {
namespace hdmap {
namespace adapter {

bool OpendriveAdapter::LoadData(const std::string& filename,
                                EDrive::hdmap::Map* pb_map) {
  CHECK_NOTNULL(pb_map);

  tinyxml2::XMLDocument document;
  if (document.LoadFile(filename.c_str()) != tinyxml2::XML_SUCCESS) {
    EERROR << "fail to load file " << filename;
    return false;
  }

  // root node
  const tinyxml2::XMLElement* root_node = document.RootElement();
  CHECK(root_node != nullptr);
  // header
  PbHeader* map_header = pb_map->mutable_header();
  Status status = HeaderXmlParser::Parse(*root_node, map_header);
  if (!status.ok()) {
    EERROR << "fail to parse opendrive header, " << status.error_message();
    return false;
  }

  // roads
  std::vector<RoadInternal> roads;
  status = RoadsXmlParser::Parse(*root_node, &roads);
  if (!status.ok()) {
    EERROR << "fail to parse opendrive road, " << status.error_message();
    return false;
  }

  // junction
  std::vector<JunctionInternal> junctions;
  status = JunctionsXmlParser::Parse(*root_node, &junctions);
  if (!status.ok()) {
    EERROR << "fail to parse opendrive junction, " << status.error_message();
    return false;
  }

  ProtoOrganizer proto_organizer;
  proto_organizer.GetRoadElements(&roads);
  proto_organizer.GetJunctionElements(junctions);
  proto_organizer.GetOverlapElements(roads, junctions);
  proto_organizer.OutputData(pb_map);

  return true;
}

}  // namespace adapter
}  // namespace hdmap
}  // namespace EDrive
