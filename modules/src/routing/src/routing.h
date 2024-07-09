/******************************************************************************
  * Copyright 2017 The EDrive Authors. All Rights Reserved.
  *
  * Licensed under the Apache License, Version 2.0 (the "License");
  * you may not use this file except in compliance with the License.
  * You may obtain a copy of the License at
  *
  * http://www.apache.org/licenses/LICENSE-2.0
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *****************************************************************************/

#pragma once

#include <memory>
#include <string>

#include "routing/proto/routing.pb.h"
#include "routing/proto/routing_config.pb.h"

#include "common/src/EDrive.h"
// #include "common/monitor_log/monitor_log_buffer.h"
#include "common/src/state.h"
#include "map/hdmap/hdmap_util.h"
#include "routing/core/navigator.h"

namespace EDrive {
namespace routing {

class Routing : public EDrive::common::EDriveApp {
  // friend class RoutingTestBase;
 public:
  Routing();

  /**
   * @brief module name
   */
  std::string Name() const override;

  /**
   * @brief module initialization function
   * @return initialization status
   */
  EDrive::common::Result_state Init() override;

  /**
   * @brief module start function
   * @return start status
   */
  EDrive::common::Result_state Start() override;

  /**
   * @brief module stop function
   */
  void Stop() override;

  /**
   * @brief destructor
   */
  virtual ~Routing() = default;

 private:
  void OnRoutingRequest(const RoutingRequest &routing_request);

  RoutingRequest FillLaneInfoIfMissing(const RoutingRequest &routing_request);

 private:
  std::unique_ptr<Navigator> navigator_ptr_;
  // EDrive::common::monitor::MonitorLogger monitor_logger_;

  RoutingConfig routing_conf_;
  const hdmap::HDMap *hdmap_ = nullptr;
};

}  // namespace routing
}  // namespace EDrive
