/******************************************************************************
 * Copyright 2024 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/
#include "map/hdmap/adapter/coordinate_convert_tool.h"
#include <math.h>
#include "glog/logging.h"

namespace EDrive {
namespace hdmap {
namespace adapter {

CoordinateConvertTool::CoordinateConvertTool()
  : pj_from_(NULL), pj_to_(NULL) {}

CoordinateConvertTool::~CoordinateConvertTool() {
  if (pj_from_) {
    pj_free(pj_from_);
    pj_from_ = NULL;
  }

  if (pj_to_) {
    pj_free(pj_to_);
    pj_to_ = NULL;
  }
}

CoordinateConvertTool* CoordinateConvertTool::GetInstance() {
  static CoordinateConvertTool instance;
  return &instance;
}

Result_state CoordinateConvertTool::SetConvertParam(const std::string &source_param,
                                             const std::string &dst_param) {
  source_convert_param_ = source_param;
  dst_convert_param_ = dst_param;
  if (pj_from_) {
      pj_free(pj_from_);
      pj_from_ = NULL;
  }

  if (pj_to_) {
    pj_free(pj_to_);
    pj_to_ = NULL;
  }

  if (!(pj_from_ = pj_init_plus(source_convert_param_.c_str()))) {
    std::string err_msg = "Fail to pj_init_plus " + source_convert_param_;
    return Result_state::State_Failed;
  }

  if (!(pj_to_ = pj_init_plus(dst_convert_param_.c_str()))) {
    std::string err_msg = "Fail to pj_init_plus " + dst_convert_param_;
    pj_free(pj_from_);
    pj_from_ = NULL;
    return Result_state::State_Failed;
  }

  return Result_state::State_Ok;
}

Result_state CoordinateConvertTool::CoordiateConvert(const double longitude,
                                             const double latitude,
                                             const double height_ellipsoid,
                                             double* utm_x, double* utm_y,
                                             double* utm_z) {
  CHECK_NOTNULL(utm_x);
  CHECK_NOTNULL(utm_y);
  CHECK_NOTNULL(utm_z);
  if (!pj_from_ || !pj_to_) {
      std::string err_msg = "no transform param";
      return Result_state::State_Failed;
  }

  double gps_longitude = longitude;
  double gps_latitude = latitude;
  double gps_alt = height_ellipsoid;

  if (pj_is_latlong(pj_from_)) {
    gps_longitude *= DEG_TO_RAD;
    gps_latitude *= DEG_TO_RAD;
    gps_alt = height_ellipsoid;
  }

  if (0 != pj_transform(pj_from_, pj_to_, 1, 1, &gps_longitude,
                          &gps_latitude, &gps_alt)) {
    std::string err_msg = "fail to transform coordinate";
    return Result_state::State_Failed;
  }

  if (pj_is_latlong(pj_to_)) {
    gps_longitude *= RAD_TO_DEG;
    gps_latitude *= RAD_TO_DEG;
  }

  *utm_x = gps_longitude;
  *utm_y = gps_latitude;
  *utm_z = gps_alt;

  return Result_state::State_Ok;
}

}  // namespace adapter
}  // namespace hdmap
}  // namespace EDrive
