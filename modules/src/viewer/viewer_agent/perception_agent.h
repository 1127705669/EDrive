/******************************************************************************
 * Copyright 2024 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#include "viewer/viewer_agent/viewer_agent_base.h"

#include <derived_object_msgs/ObjectArray.h>

namespace EDrive {
namespace viewer {

/**
 * @class PerceptionAgent
 *
 * @brief 
 */
class PerceptionAgent : public ViewerAgentBase {
 public:
  /**
   * @brief constructor
   */
  PerceptionAgent(const derived_object_msgs::ObjectArray& objects);

  /**
   * @brief destructor
   */
  virtual ~PerceptionAgent();

  /**
   * @brief initialize ViewerAgentBase
   * @param viewer_conf_ viewer configurations
   * @return Status initialization status
   */
  common::Result_state Init(const ViewerConf *viewer_conf);

  common::Result_state ProcessData() override;

  /**
   * @brief perception agent name
   * @return string agent name in string
   */
  std::string Name() const override;

 private:
  const std::string name_;
  const derived_object_msgs::ObjectArray& objects_;
};

} // namespace viewer
} // namespace EDrive