/******************************************************************************
 * Copyright 2023 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include "common/src/state.h"

#include "viewer/src/common/viewer_manager.h"
#include "viewer/proto/viewer_conf.pb.h"

namespace EDrive {
namespace viewer {

/**
 * @class ViewerAgent
 *
 * @brief manage all controllers declared in control config file.
 */
class ViewerAgent {
 public:
  /**
   * @brief initialize ViewerAgent
   * @param viewer_conf_ control configurations
   * @return Status initialization status
   */
  Result_state Init(const ViewerConf *viewer_conf_);

 private:
  /**
   * @brief
   * Register new controllers. If you need to add a new type of controller,
   * You should first register your controller in this function.
   */
  void RegisterControllers(const ViewerConf *viewer_conf_);

  std::vector<std::unique_ptr<ViewerBase>> viewer_list_;
};

} // namespace viewer
} // namespace EDrive