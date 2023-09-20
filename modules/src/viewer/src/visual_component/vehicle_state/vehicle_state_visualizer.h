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
  EDrive::Result_state Visualize(const nav_msgs::Odometry *CARLA_location, const derived_object_msgs::ObjectArray *CARLA_object, 
                                        ::viewer::VisualizingData *visualizing_data, visualization_msgs::Marker *viewer_vehicle_data) override;

 private:
  const nav_msgs::Odometry *location_ = nullptr;
  ::viewer::VisualizingData *visualizing_data_ = nullptr;

};

} // namespace viewer
} // namespace EDrive
