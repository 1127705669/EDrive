/******************************************************************************
 * Copyright 2024 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/
#pragma once

#include <tinyxml2.h>

#include "map/hdmap/adapter/xml_parser/common_define.h"
#include "map/hdmap/adapter/xml_parser/status.h"

namespace EDrive {
namespace hdmap {
namespace adapter {

class HeaderXmlParser {
 public:
  static common::Result_state Parse(const tinyxml2::XMLElement& xml_node, PbHeader* header);
};

}  // namespace adapter
}  // namespace hdmap
}  // namespace EDrive
