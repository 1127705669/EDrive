/******************************************************************************
 * Copyright 2023 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include "common/src/state.h"

#include "viewer/proto/viewer_conf.pb.h"

namespace EDrive {
namespace viewer {

/**
 * @class ViewerAgentBaseBase
 *
 * @brief manage all viewers declared in Viewer config file.
 */
class ViewerAgentBase {
 public:
  /**
   * @brief constructor
   */
  ViewerAgentBase() = default;

  /**
   * @brief destructor
   */
  ~ViewerAgentBase() = default;

  /**
   * @brief initialize ViewerAgentBase
   * @param viewer_conf_ viewer configurations
   * @return Status initialization status
   */
  virtual common::Result_state Init(const ViewerConf *viewer_conf) = 0;

  virtual common::Result_state ProcessData() = 0;

  /**
   * @brief controller name
   * @return string controller name in string
   */
  virtual std::string Name() const = 0;

 private:
  

};

} // namespace viewer
} // namespace EDrive
