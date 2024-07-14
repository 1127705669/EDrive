/******************************************************************************
 * Copyright 2024 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#include "viewer/viewer_agent/perception_agent.h"

#include "common/src/log.h"

namespace EDrive {
namespace viewer {

using EDrive::common::Result_state;

PerceptionAgent::PerceptionAgent(const derived_object_msgs::ObjectArray& objects)
 : objects_(objects), name_("Perception Viewer Agent") {
  EINFO << "Using " << name_;
}

std::string PerceptionAgent::Name() const { return name_; }

PerceptionAgent::~PerceptionAgent() {}

Result_state PerceptionAgent::Init(const ViewerConf *viewer_conf) {
  return Result_state::State_Ok;
}

Result_state PerceptionAgent::ProcessData() {
  return Result_state::State_Ok;
}

} // namespace viewer
} // namespace EDrive
