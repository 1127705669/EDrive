/******************************************************************************
 * Copyright 2023 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include "viewer/src/visual_component/viewer_base.h"
#include <nav_msgs/Odometry.h>

#include "viewer/VisualizingData.h"

/**
 * @class Vehicle_state
 *
 * @brief 
 */
namespace EDrive {
namespace viewer {

class Vehicle_state : public EDrive::viewer::ViewerBase {
 public:
/**
   * @brief 
   */
  Vehicle_state();
  

  /**
   * @brief 
   */
  common::Result_state InterfaceMatch() override;

  /**
  * @brief 
  */
  common::Result_state PublishVisualizationData() override;

  /**
  * @brief 
  */
  common::Result_state Init(const ViewerConf *viewer_conf) override;

  /**
  * @brief 
  */
  void Stop() override;

  /**
  * @brief 
  */
  common::Result_state Visualize() override;

 private:
  const nav_msgs::Odometry *location_ = nullptr;
  ::viewer::VisualizingData *visualizing_data_ = nullptr;

};

} // namespace viewer
} // namespace EDrive
