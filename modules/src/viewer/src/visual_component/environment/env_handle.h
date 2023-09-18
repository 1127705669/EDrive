/******************************************************************************
 * Copyright 2023 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include "viewer/src/common/viewer_manager.h"
#include <nav_msgs/Odometry.h>

#include "viewer/VisualizingData.h"

/**
 * @class Env_handle
 *
 * @brief 
 */
namespace EDrive {
namespace viewer {

class Env_handle : public EDrive::viewer::ViewerBase {
 public:
/**
   * @brief 
   */
  Env_handle();
  

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
  EDrive::Result_state Visualize(const nav_msgs::Odometry *location_, ::viewer::VisualizingData *visualizing_data_) override;

 private:
  const nav_msgs::Odometry *location_ = nullptr;
  ::viewer::VisualizingData *visualizing_data_ = nullptr;

};

} // namespace viewer
} // namespace EDrive
