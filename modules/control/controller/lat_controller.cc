/******************************************************************************
 * Copyright 2022 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#include "modules/common/status/status.h"
#include "modules/control/controller/lon_controller.h"

namespace EDrive {
namespace control {

class LatController : public Controller {

 public:

  LatController();

  virtual ~LatController();

  std::string Name() const override;

 protected:

};

} // control
} // EDrive