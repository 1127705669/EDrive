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

class UtilXmlParser {
 public:
  static common::Result_state ParseCurve(const tinyxml2::XMLElement& xml_node,
                           PbCurve* curve);
  static common::Result_state ParseGeometry(const tinyxml2::XMLElement& xml_node,
                              PbCurveSegment* curve_segment);
  static common::Result_state ParsePointSet(const tinyxml2::XMLElement& xml_node,
                              PbLineSegment* line_segment);
  static common::Result_state ParseOutline(const tinyxml2::XMLElement& xml_node,
                             PbPolygon* polygon);
  static common::Result_state ParsePoint(const tinyxml2::XMLElement& xml_node, PbPoint3D* pt);

  static std::string ToUpper(const std::string& s);

  static void WGS84ToUTM(const double x, const double y, const double z,
                         double* output_x, double* output_y, double* output_z);

  static double CurveLength(const PbCurve& curve);

  static tinyxml2::XMLError QueryStringAttribute(
      const tinyxml2::XMLElement& xml_node, const std::string& name,
      std::string* value);
};

}  // namespace adapter
}  // namespace hdmap
}  // namespace EDrive
