/******************************************************************************
 * Copyright 2024 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/
#pragma once

#include <string>

#include "map/proto/map.pb.h"

#include "map/hdmap/adapter/coordinate_convert_tool.h"
#include "map/hdmap/adapter/xml_parser/common_define.h"
#include "map/hdmap/adapter/xml_parser/header_xml_parser.h"
#include "map/hdmap/adapter/xml_parser/junctions_xml_parser.h"
#include "map/hdmap/adapter/xml_parser/lanes_xml_parser.h"
#include "map/hdmap/adapter/xml_parser/objects_xml_parser.h"
#include "map/hdmap/adapter/xml_parser/roads_xml_parser.h"
#include "map/hdmap/adapter/xml_parser/signals_xml_parser.h"

namespace EDrive {
namespace hdmap {
namespace adapter {

class OpendriveAdapter {
 public:
  static bool LoadData(const std::string& filename, EDrive::hdmap::Map* pb_map);
};

}  // namespace adapter
}  // namespace hdmap
}  // namespace EDrive
