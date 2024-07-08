/******************************************************************************
  * Copyright 2024 The EDrive Authors. All Rights Reserved.
  *****************************************************************************/

#include "routing/topo_creator/edge_creator.h"

#include <math.h>

namespace EDrive {
namespace routing {

void EdgeCreator::GetPbEdge(const Node& node_from, const Node& node_to,
                            const Edge::DirectionType& type, Edge* pb_edge,
                            const RoutingConfig* routing_config) {
  InitEdgeInfo(node_from, node_to, type, pb_edge, routing_config);
  InitEdgeCost(node_from, node_to, type, pb_edge, routing_config);
}

void EdgeCreator::InitEdgeInfo(const Node& node_from, const Node& node_to,
                               const Edge::DirectionType& type,
                               Edge* const edge,
                               const RoutingConfig* routing_config) {
  edge->set_from_lane_id(node_from.lane_id());
  edge->set_to_lane_id(node_to.lane_id());
  edge->set_direction_type(type);
}

void EdgeCreator::InitEdgeCost(const Node& node_from, const Node& node_to,
                               const Edge::DirectionType& type,
                               Edge* const edge,
                               const RoutingConfig* routing_config) {
  edge->set_cost(0.0);
  if (type == Edge::LEFT || type == Edge::RIGHT) {
    const auto& target_range =
        (type == Edge::LEFT) ? node_from.left_out() : node_from.right_out();
    double changing_area_length = 0.0;
    for (const auto& range : target_range) {
      changing_area_length += range.end().s() - range.start().s();
    }
    double ratio =
        (changing_area_length >= routing_config->base_changing_length())
            ? pow(changing_area_length / routing_config->base_changing_length(),
                  -1.5)
            : 1.0;
    edge->set_cost(routing_config->change_penalty() * ratio);
  }
}

}  // namespace routing
}  // namespace EDrive
