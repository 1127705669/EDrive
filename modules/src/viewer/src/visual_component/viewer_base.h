/******************************************************************************
 * Copyright 2023 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include "common/src/state.h"
#include "viewer/proto/viewer_conf.pb.h"
#include <nav_msgs/Odometry.h>
#include "viewer/VisualizingData.h"
#include "derived_object_msgs/ObjectArray.h"

namespace EDrive {
namespace viewer {

/**
 * @class ViewerBase
 * @brief Base interface of all viewer component.
 */
class ViewerBase {
 public:

  /**
   * @brief initialize Controller
   * @param control_conf control configurations
   * @return Status initialization status
   */
  virtual Result_state Init(const ViewerConf *viewer_conf) = 0;
  
  virtual ~ViewerBase() = default;

  virtual EDrive::Result_state InterfaceMatch() = 0;

  virtual EDrive::Result_state PublishVisualizationData() = 0;

  virtual EDrive::Result_state Visualize(const nav_msgs::Odometry *location, const derived_object_msgs::ObjectArray *object, ::viewer::VisualizingData *visualizing_data) = 0;

  virtual void Stop() = 0;

};

} // namespace viewer
} // namespace EDrive