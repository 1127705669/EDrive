/******************************************************************************
 * Copyright 2023 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include "viewer/src/visual_component/viewer_base.h"
#include <nav_msgs/Odometry.h>

#include "viewer/VisualizingData.h"
#include "derived_object_msgs/ObjectArray.h"
#include <visualization_msgs/MarkerArray.h>

/**
 * @class Env_handle
 *
 * @brief 
 */
namespace EDrive {
namespace viewer {

class Env_handle : public EDrive::viewer::ViewerBase{
 public:
  /**
   * @brief 
   */
  Env_handle(derived_object_msgs::ObjectArray *objects, visualization_msgs::MarkerArray *objects_marker_array);
  
  /**
   * @brief 
   */
  EDrive::common::Result_state InterfaceMatch() override;

  /**
  * @brief 
  */
  EDrive::common::Result_state PublishVisualizationData() override;

  /**
  * @brief 
  */
  EDrive::common::Result_state Init(const ViewerConf *viewer_conf) override;

  /**
  * @brief 
  */
  void Stop() override;

  /**
  * @brief 
  */
  EDrive::common::Result_state Visualize() override;

 private:
  const nav_msgs::Odometry *location_ = nullptr;
  ::viewer::VisualizingData *visualizing_data_ = nullptr;
  derived_object_msgs::ObjectArray *objects_ = nullptr;
  visualization_msgs::MarkerArray *objects_marker_array_;

};

} // namespace viewer
} // namespace EDrive
