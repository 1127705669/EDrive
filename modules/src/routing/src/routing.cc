/******************************************************************************
 * Copyright 2022 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#include "routing/src/routing.h"

#include "common/adapters/adapter_manager.h"
#include "routing/common/routing_gflags.h"

namespace EDrive {
namespace routing {

using EDrive::common::Result_state;

std::string Routing::Name() const { return "routing"; }

Result_state Routing::Init(){
  // const auto routing_map_file = EDrive::hdmap::RoutingMapFile();
  // AINFO << "Use routing topology graph path: " << routing_map_file;
  // navigator_ptr_.reset(new Navigator(routing_map_file));
  // CHECK(common::util::GetProtoFromFile(FLAGS_routing_conf_file, &routing_conf_))
  //     << "Unable to load routing conf file: " + FLAGS_routing_conf_file;

  // AINFO << "Conf file: " << FLAGS_routing_conf_file << " is loaded.";

  // hdmap_ = EDrive::hdmap::HDMapUtil::BaseMapPtr();
  // CHECK(hdmap_) << "Failed to load map file:" << EDrive::hdmap::BaseMapFile();

  // AdapterManager::Init(FLAGS_routing_adapter_config_filename);
  // AdapterManager::AddRoutingRequestCallback(&Routing::OnRoutingRequest, this);
  // return EDrive::common::Status::OK();
  return Result_state::State_Ok;
}

Result_state Routing::Start(){
  return Result_state::State_Ok;
}

void Routing::Stop() {
  
}

} // namespace routing
} // namespace EDrive