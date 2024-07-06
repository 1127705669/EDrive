/******************************************************************************
 * Copyright 2024 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/
#include "map/hdmap/adapter/xml_parser/util_xml_parser.h"

#include <algorithm>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include "common/src/log.h"
#include "map/hdmap/adapter/coordinate_convert_tool.h"

namespace EDrive {
namespace hdmap {
namespace adapter {

Result_state UtilXmlParser::ParseCurve(const tinyxml2::XMLElement& xml_node,
                                 PbCurve* curve) {
  CHECK_NOTNULL(curve);

  const tinyxml2::XMLElement* sub_node = xml_node.FirstChildElement("geometry");
  while (sub_node) {
    PbCurveSegment* curve_segment = curve->add_segment();
    RETURN_IF_ERROR(UtilXmlParser::ParseGeometry(*sub_node, curve_segment));
    sub_node = sub_node->NextSiblingElement("geometry");
  }

  return Result_state::State_Ok;
}

Result_state UtilXmlParser::ParseGeometry(const tinyxml2::XMLElement& xml_node,
                                    PbCurveSegment* curve_segment) {
  CHECK_NOTNULL(curve_segment);

  // Read geometry attributes
  double s = 0.0;
  double ptx = 0.0;
  double pty = 0.0;
  double ptz = 0.0;
  double length = 0.0;

  int checker = tinyxml2::XML_SUCCESS;

  checker += xml_node.QueryDoubleAttribute("sOffset", &s);
  checker += xml_node.QueryDoubleAttribute("x", &ptx);
  checker += xml_node.QueryDoubleAttribute("y", &pty);
  checker += xml_node.QueryDoubleAttribute("length", &length);

  if (checker == tinyxml2::XML_SUCCESS) {
    curve_segment->set_s(s);

    double output_x = 0.0;
    double output_y = 0.0;
    double output_z = 0.0;

    WGS84ToUTM(ptx, pty, ptz, &output_x, &output_y, &output_z);

    curve_segment->mutable_start_position()->set_x(output_x);
    curve_segment->mutable_start_position()->set_y(output_y);
    curve_segment->set_length(length);
  }

  const auto sub_node = xml_node.FirstChildElement("pointSet");
  if (sub_node) {
    PbLineSegment* line_segment = curve_segment->mutable_line_segment();
    RETURN_IF_ERROR(ParsePointSet(*sub_node, line_segment));
    return Result_state::State_Ok;
  }

  std::string err_msg = "Error geometry object";
  return Result_state::State_Failed;
}

Result_state UtilXmlParser::ParsePointSet(const tinyxml2::XMLElement& xml_node,
                                    PbLineSegment* line_segment) {
  const tinyxml2::XMLElement* sub_node = xml_node.FirstChildElement("point");
  while (sub_node) {
    double ptx = 0.0;
    double pty = 0.0;
    double ptz = 0.0;
    int checker = tinyxml2::XML_SUCCESS;
    checker += sub_node->QueryDoubleAttribute("x", &ptx);
    checker += sub_node->QueryDoubleAttribute("y", &pty);

    if (checker != tinyxml2::XML_SUCCESS) {
      std::string err_msg = "Error parsing geometry point attributes";
      return Result_state::State_Failed;
    }

    PbPoint3D* pt = line_segment->add_point();
    double output_x = 0.0;
    double output_y = 0.0;
    double output_z = 0.0;
    WGS84ToUTM(ptx, pty, ptz, &output_x, &output_y, &output_z);
    pt->set_x(output_x);
    pt->set_y(output_y);

    sub_node = sub_node->NextSiblingElement("point");
  }

  return Result_state::State_Ok;
}

Result_state UtilXmlParser::ParseOutline(const tinyxml2::XMLElement& xml_node,
                                   PbPolygon* polygon) {
  const tinyxml2::XMLElement* sub_node =
      xml_node.FirstChildElement("cornerGlobal");
  while (sub_node) {
    double ptx = 0.0;
    double pty = 0.0;
    double ptz = 0.0;
    int checker = tinyxml2::XML_SUCCESS;
    checker += sub_node->QueryDoubleAttribute("x", &ptx);
    checker += sub_node->QueryDoubleAttribute("y", &pty);
    checker += sub_node->QueryDoubleAttribute("z", &ptz);

    if (checker != tinyxml2::XML_SUCCESS) {
      std::string err_msg = "Error parsing cornerGlobal point attributes";
      return Result_state::State_Failed;
    }

    PbPoint3D* pt = polygon->add_point();
    double output_x = 0.0;
    double output_y = 0.0;
    double output_z = 0.0;
    WGS84ToUTM(ptx, pty, ptz, &output_x, &output_y, &output_z);
    pt->set_x(output_x);
    pt->set_y(output_y);
    pt->set_z(output_z);

    sub_node = sub_node->NextSiblingElement("cornerGlobal");
  }

  return Result_state::State_Ok;
}

Result_state UtilXmlParser::ParsePoint(const tinyxml2::XMLElement& xml_node,
                                 PbPoint3D* pt) {
  CHECK_NOTNULL(pt);

  const auto sub_node = xml_node.FirstChildElement("centerPoint");
  CHECK(sub_node != nullptr);
  int checker = tinyxml2::XML_SUCCESS;
  double ptx = 0.0;
  double pty = 0.0;
  double ptz = 0.0;
  checker += sub_node->QueryDoubleAttribute("x", &ptx);
  checker += sub_node->QueryDoubleAttribute("y", &pty);
  checker += sub_node->QueryDoubleAttribute("z", &ptz);

  if (checker != tinyxml2::XML_SUCCESS) {
    std::string err_msg = "Error parse point attributes";
    return Result_state::State_Failed;
  }

  double output_x = 0.0;
  double output_y = 0.0;
  double output_z = 0.0;
  WGS84ToUTM(ptx, pty, ptz, &output_x, &output_y, &output_z);
  pt->set_x(output_x);
  pt->set_y(output_y);
  pt->set_z(output_z);

  return Result_state::State_Ok;
}

std::string UtilXmlParser::ToUpper(const std::string& s) {
  std::string value = s;
  std::transform(value.begin(), value.end(), value.begin(),
                 [](unsigned char c) { return std::toupper(c); });

  return value;
}

void UtilXmlParser::WGS84ToUTM(const double x, const double y, const double z,
                               double* output_x, double* output_y,
                               double* output_z) {
  CoordinateConvertTool::GetInstance()->CoordiateConvert(x, y, z, output_x,
                                                         output_y, output_z);
}

double UtilXmlParser::CurveLength(const PbCurve& curve) {
  double length = 0.0;
  for (int i = 0; i < curve.segment_size(); ++i) {
    length += curve.segment(i).length();
  }

  return length;
}

tinyxml2::XMLError UtilXmlParser::QueryStringAttribute(
    const tinyxml2::XMLElement& xml_node, const std::string& name,
    std::string* value) {
  CHECK_NOTNULL(value);
  const char* val = xml_node.Attribute(name.c_str());
  if (val == nullptr) {
    return tinyxml2::XML_NO_ATTRIBUTE;
  }

  *value = val;
  return tinyxml2::XML_SUCCESS;
}

}  // namespace adapter
}  // namespace hdmap
}  // namespace EDrive
