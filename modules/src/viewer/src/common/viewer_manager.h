/******************************************************************************
 * Copyright 2023 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include "common/src/state.h"
#include "viewer/proto/viewer_conf.pb.h"
#include <nav_msgs/Odometry.h>
#include "viewer/VisualizingData.h"

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
  virtual Result_state Init(const ViewerConf *viewer_conf_) = 0;
  
  virtual ~ViewerBase() = default;

  virtual EDrive::Result_state InterfaceMatch() = 0;

  virtual EDrive::Result_state PublishVisualizationData() = 0;

  virtual EDrive::Result_state Visualize(const nav_msgs::Odometry *location_, ::viewer::VisualizingData *visualizing_data_) = 0;
  
 protected:

  virtual void Stop();

};

} // namespace viewer
} // namespace EDrive