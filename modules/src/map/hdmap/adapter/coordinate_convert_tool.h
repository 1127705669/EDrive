/******************************************************************************
 * Copyright 2024 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/
#pragma once

#define ACCEPT_USE_OF_DEPRECATED_PROJ_API_H
#include <proj_api.h>
#include <string>
#include "map/hdmap/adapter/xml_parser/status.h"

namespace EDrive {
namespace hdmap  {
namespace adapter {

class CoordinateConvertTool {
 public:
  CoordinateConvertTool();
  ~CoordinateConvertTool();

 public:
  static CoordinateConvertTool* GetInstance();

 public:
  common::Result_state SetConvertParam(const std::string &source_param,
                        const std::string &dst_param);
  common::Result_state CoordiateConvert(const double longitude, const double latitude,
                          const double height_ellipsoid, double* utm_x,
                          double* utm_y, double* utm_z);

 private:
  std::string source_convert_param_;
  std::string dst_convert_param_;

  projPJ pj_from_;
  projPJ pj_to_;
};

}  // namespace adapter
}  // namespace hdmap
}  // namespace EDrive
