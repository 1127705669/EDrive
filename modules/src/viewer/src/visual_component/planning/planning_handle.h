/******************************************************************************
 * Copyright 2023 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include "viewer/src/visual_component/viewer_base.h"

#include "viewer/VisualizingData.h"

#include <nav_msgs/Path.h>
#include "planning/ADCTrajectory.h"

/**
 * @class Planning_handle
 *
 * @brief 
 */
namespace EDrive {
namespace viewer {

class Planning_handle : public EDrive::viewer::ViewerBase{
 public:
  /**
   * @brief 
   */
  Planning_handle(::planning::ADCTrajectory *trajectory, nav_msgs::Path *trajectory_path);
  
  /**
   * @brief 
   */
  EDrive::Result_state InterfaceMatch() override;

  /**
  * @brief 
  */
  EDrive::Result_state PublishVisualizationData() override;

  /**
  * @brief 
  */
  EDrive::Result_state Init(const ViewerConf *viewer_conf) override;

  /**
  * @brief 
  */
  void Stop() override;

  /**
  * @brief 
  */
  EDrive::Result_state Visualize() override;

 private:
  ::viewer::VisualizingData *visualizing_data_ = nullptr;
  ::planning::ADCTrajectory *trajectory_ = nullptr;
  nav_msgs::Path *trajectory_path_ = nullptr;

};

} // namespace viewer
} // namespace EDrive
