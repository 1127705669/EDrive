/******************************************************************************
 * Copyright 2023 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include "viewer/src/common/viewer_manager.h"


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

};

} // namespace viewer
} // namespace EDrive