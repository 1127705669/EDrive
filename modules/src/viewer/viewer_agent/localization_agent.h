/******************************************************************************
 * Copyright 2024 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#include "viewer/viewer_agent/viewer_agent_base.h"

#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>

#include "common/configs/proto/vehicle_config.pb.h"

namespace EDrive {
namespace viewer {

/**
 * @class LocalizationAgent
 *
 * @brief 
 */
class LocalizationAgent : public ViewerAgentBase {
 public:
  /**
   * @brief constructor
   */
  LocalizationAgent(const nav_msgs::Odometry& ego_vehicle_localization, visualization_msgs::Marker& ego_vehicle_marker);

  /**
   * @brief destructor
   */
  virtual ~LocalizationAgent();

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
  void VisualizeLocalization();
  const std::string name_;

  // vehicle parameter
  common::VehicleParam vehicle_param_;

  const nav_msgs::Odometry& ego_vehicle_odometry_;
  visualization_msgs::Marker &ego_vehicle_marker_;
};

} // namespace viewer
} // namespace EDrive