/******************************************************************************
 * Copyright 2024 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#include "map/hdmap/adapter/xml_parser/header_xml_parser.h"

#include <string>

#include "map/hdmap/adapter/coordinate_convert_tool.h"
#include "map/hdmap/adapter/xml_parser/util_xml_parser.h"

namespace {
int GetLongZone(double longitude) {
  double longZone = 0.0;
  if (longitude < 0.0) {
    longZone = ((180.0 + longitude) / 6.0) + 1;
  } else {
    longZone = (longitude / 6.0) + 31;
  }
  return static_cast<int>(longZone);
}
}  // namespace

namespace EDrive {
namespace hdmap {
namespace adapter {

Result_state HeaderXmlParser::Parse(const tinyxml2::XMLElement& xml_node,
                              PbHeader* header) {
  auto header_node = xml_node.FirstChildElement("header");
  if (!header_node) {
    std::string err_msg = "xml data missing header";
    return Result_state::State_Failed;
  }
  std::string rev_major;
  std::string rev_minor;
  std::string database_name;
  std::string version;
  std::string date;
  double north = 0.0;
  double south = 0.0;
  double west = 0.0;
  double east = 0.0;
  std::string vendor;
  int checker = UtilXmlParser::QueryStringAttribute(
                            *header_node, "revMajor", &rev_major);
  checker += UtilXmlParser::QueryStringAttribute(
                            *header_node, "revMinor", &rev_minor);
  checker += UtilXmlParser::QueryStringAttribute(
                            *header_node, "name", &database_name);
  checker += UtilXmlParser::QueryStringAttribute(
                            *header_node, "version", &version);
  checker += UtilXmlParser::QueryStringAttribute(
                            *header_node, "date", &date);
  checker += header_node->QueryDoubleAttribute("north", &north);
  checker += header_node->QueryDoubleAttribute("south", &south);
  checker += header_node->QueryDoubleAttribute("east", &east);
  checker += header_node->QueryDoubleAttribute("west", &west);
  checker +=
      UtilXmlParser::QueryStringAttribute(*header_node, "vendor", &vendor);

  if (checker != tinyxml2::XML_SUCCESS) {
    std::string err_msg = "Error parsing header attributes";
    return Result_state::State_Failed;
  }

  auto geo_reference_node = header_node->FirstChildElement("geoReference");
  if (!geo_reference_node) {
    std::string err_msg = "Error parsing header geoReoference attributes";
    return Result_state::State_Failed;
  }
  auto geo_text = geo_reference_node->FirstChild()->ToText();
  if (!geo_text) {
    std::string err_msg = "Error parsing header geoReoference text";
    return Result_state::State_Failed;
  }

  // coordinate frame
  std::string from_coordinate = geo_text->Value();
  int eastZone = GetLongZone(east);
  int westZone = GetLongZone(west);
  if (eastZone != westZone) {
    std::string err_msg = "unsupport data in more than one zones";
    return Result_state::State_Failed;
  }
  int zone = westZone;
  std::string to_coordinate = "+proj=utm +zone=" + std::to_string(zone) +
                              " +ellps=WGS84 +datum=WGS84 +units=m +no_defs";
  CoordinateConvertTool::GetInstance()->SetConvertParam(from_coordinate,
                                                        to_coordinate);

  header->set_version(version);
  header->set_date(date);
  header->mutable_projection()->set_proj(to_coordinate);
  header->set_district(database_name);
  header->set_rev_major(rev_major);
  header->set_rev_minor(rev_minor);
  header->set_left(west);
  header->set_right(east);
  header->set_top(north);
  header->set_bottom(south);
  header->set_vendor(vendor);

  return Result_state::State_Ok;
}

}  // namespace adapter
}  // namespace hdmap
}  // namespace EDrive
