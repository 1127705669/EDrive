/******************************************************************************
 * Copyright 2024 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/
#include "map/hdmap/adapter/xml_parser/junctions_xml_parser.h"

#include <string>
#include <vector>

#include <tinyxml2.h>

#include "map/hdmap/adapter/xml_parser/common_define.h"
#include "map/hdmap/adapter/xml_parser/status.h"
#include "map/hdmap/adapter/xml_parser/util_xml_parser.h"

namespace EDrive {
namespace hdmap {
namespace adapter {

Result_state JunctionsXmlParser::Parse(const tinyxml2::XMLElement& xml_node,
                                 std::vector<JunctionInternal>* junctions) {
  const tinyxml2::XMLElement* junction_node =
      xml_node.FirstChildElement("junction");
  while (junction_node) {
    // id
    std::string junction_id;
    int checker =
        UtilXmlParser::QueryStringAttribute(*junction_node, "id", &junction_id);
    if (checker != tinyxml2::XML_SUCCESS) {
      std::string err_msg = "Error parse junction id";
      return Result_state::State_Failed;
    }

    // outline
    const tinyxml2::XMLElement* sub_node =
        junction_node->FirstChildElement("outline");
    if (!sub_node) {
      std::string err_msg = "Error parse junction outline";
      return Result_state::State_Failed;
    }

    PbJunction junction;
    junction.mutable_id()->set_id(junction_id);
    PbPolygon* polygon = junction.mutable_polygon();
    RETURN_IF_ERROR(UtilXmlParser::ParseOutline(*sub_node, polygon));

    JunctionInternal junction_internal;
    junction_internal.junction = junction;

    // overlap
    sub_node = junction_node->FirstChildElement("objectOverlapGroup");
    if (sub_node) {
      sub_node = sub_node->FirstChildElement("objectReference");
      while (sub_node) {
        std::string object_id;
        checker =
            UtilXmlParser::QueryStringAttribute(*sub_node, "id", &object_id);
        if (checker != tinyxml2::XML_SUCCESS) {
          std::string err_msg = "Error parse junction overlap id";
          return Result_state::State_Failed;
        }

        OverlapWithJunction overlap_with_juntion;
        overlap_with_juntion.object_id = object_id;
        junction_internal.overlap_with_junctions.push_back(
            overlap_with_juntion);

        sub_node = sub_node->NextSiblingElement("objectReference");
      }
    }

    junctions->push_back(junction_internal);
    junction_node = junction_node->NextSiblingElement("junction");
  }
  return Result_state::State_Ok;
}

}  // namespace adapter
}  // namespace hdmap
}  // namespace EDrive
