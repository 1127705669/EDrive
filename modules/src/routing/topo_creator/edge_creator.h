/******************************************************************************
  * Copyright 2024 The EDrive Authors. All Rights Reserved.
  *****************************************************************************/

#pragma once

#include "routing/proto/routing_config.pb.h"
#include "routing/proto/topo_graph.pb.h"

namespace EDrive {
namespace routing {

class EdgeCreator {
 public:
  static void GetPbEdge(const Node& node_from, const Node& node_to,
                        const Edge::DirectionType& type, Edge* pb_edge,
                        const RoutingConfig* routingconfig);

 private:
  static void InitEdgeInfo(const Node& node_from, const Node& node_to,
                           const Edge::DirectionType& type, Edge* pb_edge,
                           const RoutingConfig* routingconfig);
  static void InitEdgeCost(const Node& node_from, const Node& node_to,
                           const Edge::DirectionType& type, Edge* pb_edge,
                           const RoutingConfig* routingconfig);
};

}  // namespace routing
}  // namespace EDrive
