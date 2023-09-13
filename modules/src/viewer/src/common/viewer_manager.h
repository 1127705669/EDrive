/******************************************************************************
 * Copyright 2023 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include "common/src/state.h"

namespace EDrive {
namespace viewer {

/**
 * @class ViewerBase
 * @brief Base interface of all viewer component.
 */
class ViewerBase {
 public:
  
  virtual ~ViewerBase() = default;

  virtual EDrive::Result_state InterfaceMatch() = 0;

  virtual EDrive::Result_state PublishVisualizationData() = 0;
  
 protected:
  
  virtual EDrive::Result_state Init() = 0;

  virtual void Stop();

};

} // namespace viewer
} // namespace EDrive