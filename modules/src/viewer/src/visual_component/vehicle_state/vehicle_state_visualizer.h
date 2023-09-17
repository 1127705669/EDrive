/******************************************************************************
 * Copyright 2023 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include "viewer/src/common/viewer_manager.h"
#include <nav_msgs/Odometry.h>


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
  EDrive::Result_state InterfaceMatch() override;

  /**
  * @brief 
  */
  EDrive::Result_state PublishVisualizationData() override;

  /**
  * @brief 
  */
  EDrive::Result_state Init(const ViewerConf *viewer_conf_) override;

  /**
  * @brief 
  */
  void Stop() override;

  /**
  * @brief 
  */
  EDrive::Result_state Visualize(const nav_msgs::Odometry *location_, ::viewer::VisualizingData *visualizing_data_) override;

 private:
  std::unique_ptr<nav_msgs::Odometry> location_message_;
  std::unique_ptr<::viewer::VisualizingData>  visualizing_message_;

};

} // namespace viewer
} // namespace EDrive
