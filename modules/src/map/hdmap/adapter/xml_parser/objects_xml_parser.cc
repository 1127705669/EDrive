/******************************************************************************
 * Copyright 2024 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#include "map/hdmap/adapter/xml_parser/objects_xml_parser.h"

#include <string>
#include <vector>

#include "map/hdmap/adapter/xml_parser/util_xml_parser.h"

namespace EDrive {
namespace hdmap {
namespace adapter {

Result_state ObjectsXmlParser::ParseCrosswalks(const tinyxml2::XMLElement& xml_node,
                                         std::vector<PbCrosswalk>* crosswalks) {
  CHECK_NOTNULL(crosswalks);
  const tinyxml2::XMLElement* sub_node = xml_node.FirstChildElement("object");
  while (sub_node) {
    std::string object_type;
    std::string object_id;
    int checker =
        UtilXmlParser::QueryStringAttribute(*sub_node, "type", &object_type);
    checker += UtilXmlParser::QueryStringAttribute(*sub_node, "id", &object_id);
    if (checker != tinyxml2::XML_SUCCESS) {
      std::string err_msg = "Error parse object type.";
      return Result_state::State_Failed;
    }

    if (object_type == "crosswalk") {
      PbCrosswalk crosswalk;
      crosswalk.mutable_id()->set_id(object_id);
      PbPolygon* polygon = crosswalk.mutable_polygon();
      const tinyxml2::XMLElement* outline_node =
          sub_node->FirstChildElement("outline");
      if (outline_node == nullptr) {
        std::string err_msg = "Error parse crosswalk outline";
        return Result_state::State_Failed;
      }
      RETURN_IF_ERROR(UtilXmlParser::ParseOutline(*outline_node, polygon));
      crosswalks->emplace_back(crosswalk);
    }
    sub_node = sub_node->NextSiblingElement("object");
  }
  return Result_state::State_Ok;
}

Result_state ObjectsXmlParser::ParseClearAreas(
    const tinyxml2::XMLElement& xml_node,
    std::vector<PbClearArea>* clear_areas) {
  CHECK_NOTNULL(clear_areas);
  const tinyxml2::XMLElement* sub_node = xml_node.FirstChildElement("object");
  while (sub_node) {
    std::string object_type;
    std::string object_id;
    int checker =
        UtilXmlParser::QueryStringAttribute(*sub_node, "id", &object_id);
    checker +=
        UtilXmlParser::QueryStringAttribute(*sub_node, "type", &object_type);
    if (checker != tinyxml2::XML_SUCCESS) {
      std::string err_msg = "Error parse object type.";
      return Result_state::State_Failed;
    }

    if (object_type == "clearArea") {
      PbClearArea clear_area;
      clear_area.mutable_id()->set_id(object_id);
      PbPolygon* polygon = clear_area.mutable_polygon();
      CHECK(polygon != nullptr);
      const tinyxml2::XMLElement* outline_node =
          sub_node->FirstChildElement("outline");
      if (outline_node == nullptr) {
        std::string err_msg = "Error parse cleararea outline";
        return Result_state::State_Failed;
      }
      RETURN_IF_ERROR(UtilXmlParser::ParseOutline(*outline_node, polygon));
      clear_areas->emplace_back(clear_area);
    }
    sub_node = sub_node->NextSiblingElement("object");
  }

  return Result_state::State_Ok;
}

Result_state ObjectsXmlParser::ParseSpeedBumps(
    const tinyxml2::XMLElement& xml_node,
    std::vector<PbSpeedBump>* speed_bumps) {
  CHECK_NOTNULL(speed_bumps);
  const tinyxml2::XMLElement* object_node =
      xml_node.FirstChildElement("object");
  while (object_node) {
    std::string object_type;
    std::string object_id;
    int checker =
        UtilXmlParser::QueryStringAttribute(*object_node, "id", &object_id);
    checker +=
        UtilXmlParser::QueryStringAttribute(*object_node, "type", &object_type);
    if (checker != tinyxml2::XML_SUCCESS) {
      std::string err_msg = "Error parse object type.";
      return Result_state::State_Failed;
    }

    if (object_type == "speedBump") {
      PbSpeedBump speed_bump;
      const tinyxml2::XMLElement* sub_node =
          object_node->FirstChildElement("geometry");
      speed_bump.mutable_id()->set_id(object_id);
      while (sub_node) {
        PbCurve* curve = speed_bump.add_position();
        PbCurveSegment* curve_segment = curve->add_segment();
        RETURN_IF_ERROR(UtilXmlParser::ParseGeometry(*sub_node, curve_segment));
        sub_node = sub_node->NextSiblingElement("geometry");
      }
      if (speed_bump.position_size() <= 0) {
        std::string err_msg = "Error speed bump miss stop line.";
        return Result_state::State_Failed;
      }
      speed_bumps->emplace_back(speed_bump);
    }
    object_node = object_node->NextSiblingElement("object");
  }
  return Result_state::State_Ok;
}

Result_state ObjectsXmlParser::ParseStopLines(
    const tinyxml2::XMLElement& xml_node,
    std::vector<StopLineInternal>* stop_lines) {
  CHECK_NOTNULL(stop_lines);
  const tinyxml2::XMLElement* object_node =
      xml_node.FirstChildElement("object");
  while (object_node) {
    std::string object_type;
    std::string object_id;
    int checker =
        UtilXmlParser::QueryStringAttribute(*object_node, "id", &object_id);
    checker +=
        UtilXmlParser::QueryStringAttribute(*object_node, "type", &object_type);
    if (checker != tinyxml2::XML_SUCCESS) {
      std::string err_msg = "Error parse object type.";
      return Result_state::State_Failed;
    }

    if (object_type == "stopline") {
      StopLineInternal stop_line;
      stop_line.id = object_id;
      PbCurveSegment* curve_segment = stop_line.curve.add_segment();
      CHECK(curve_segment != nullptr);
      const auto sub_node = object_node->FirstChildElement("geometry");
      if (sub_node == nullptr) {
        std::string err_msg = "Error parse stopline geometry";
        return Result_state::State_Failed;
      }
      RETURN_IF_ERROR(UtilXmlParser::ParseGeometry(*sub_node, curve_segment));
      stop_lines->emplace_back(stop_line);
    }
    object_node = object_node->NextSiblingElement("object");
  }
  return Result_state::State_Ok;
}

Result_state ObjectsXmlParser::ParseParkingSpaces(
        const tinyxml2::XMLElement& xml_node,
        std::vector<PbParkingSpace>* parking_spaces) {
  CHECK_NOTNULL(parking_spaces);
  const tinyxml2::XMLElement* sub_node = xml_node.FirstChildElement("object");
  while (sub_node) {
    std::string object_type;
    std::string object_id;
    int checker =
        UtilXmlParser::QueryStringAttribute(*sub_node, "type", &object_type);
    checker += UtilXmlParser::QueryStringAttribute(*sub_node, "id", &object_id);
    if (checker != tinyxml2::XML_SUCCESS) {
      std::string err_msg = "Error parse object type.";
      return Result_state::State_Failed;
    }

    if (object_type == "parkingSpace") {
      PbParkingSpace parking_space;
      parking_space.mutable_id()->set_id(object_id);

      double heading = 0.0;
      checker = sub_node->QueryDoubleAttribute("heading", &heading);
      if (checker != tinyxml2::XML_SUCCESS) {
        std::string err_msg = "Error parse parking space heading.";
        return Result_state::State_Failed;
      }
      parking_space.set_heading(heading);

      PbPolygon* polygon = parking_space.mutable_polygon();
      const auto* outline_node = sub_node->FirstChildElement("outline");
      if (outline_node == nullptr) {
        std::string err_msg = "Error parse parking space outline";
        return Result_state::State_Failed;
      }
      RETURN_IF_ERROR(UtilXmlParser::ParseOutline(*outline_node, polygon));
      parking_spaces->emplace_back(parking_space);
    }
    sub_node = sub_node->NextSiblingElement("object");
  }
  return Result_state::State_Ok;
}

}  // namespace adapter
}  // namespace hdmap
}  // namespace EDrive
