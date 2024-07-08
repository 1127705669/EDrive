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

#include "common/src/EDrive.h"
#include "common/configs/config_gflags.h"
#include "common/src/log.h"
#include "common/util/file.h"
#include "map/hdmap/hdmap_util.h"
#include "routing/common/routing_gflags.h"
#include "routing/proto/routing_config.pb.h"
#include "routing/topo_creator/graph_creator.h"

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  EDrive::routing::RoutingConfig routing_conf;

  CHECK(EDrive::common::util::GetProtoFromFile(FLAGS_routing_conf_file,
                                               &routing_conf))
      << "Unable to load routing conf file: " + FLAGS_routing_conf_file;

  EINFO << "Conf file: " << FLAGS_routing_conf_file << " is loaded.";

  const auto base_map = EDrive::hdmap::BaseMapFile();
  const auto routing_map = EDrive::hdmap::RoutingMapFile();
  EDrive::routing::GraphCreator creator(base_map, routing_map, &routing_conf);
  CHECK(creator.Create()) << "Create routing topo failed!";

  EINFO << "Create routing topo successfully from " << base_map << " to "
        << routing_map;
  return 0;
}
