/******************************************************************************
  * Copyright 2024 The EDirve Authors. All Rights Reserved.
  *****************************************************************************/

#include "routing/topo_creator/node_creator.h"

#include <algorithm>
#include <cmath>

namespace EDrive {
namespace routing {

namespace {

using ::google::protobuf::RepeatedPtrField;

using EDrive::hdmap::Lane;
using EDrive::hdmap::LaneBoundary;
using EDrive::hdmap::LaneBoundaryType;

bool IsAllowedOut(const LaneBoundaryType& type) {
  if (type.types(0) == LaneBoundaryType::DOTTED_YELLOW ||
      type.types(0) == LaneBoundaryType::DOTTED_WHITE) {
    return true;
  }
  return false;
}

double GetLengthbyRate(double cur_s, double cur_total_length,
                       double target_length) {
  double new_length = cur_s / cur_total_length * target_length;
  return std::min(new_length, target_length);
}

double GetLaneLength(const Lane& lane) {
  double length = 0.0;
  for (const auto& segment : lane.central_curve().segment()) {
    length += segment.length();
  }
  return length;
}

}  // namespace

void NodeCreator::GetPbNode(const Lane& lane, const std::string& road_id,
                            Node* pb_node, const RoutingConfig* routingconfig) {
  InitNodeInfo(lane, road_id, pb_node, routingconfig);
  InitNodeCost(lane, pb_node, routingconfig);
}

void NodeCreator::AddOutBoundary(
    const LaneBoundary& bound, double lane_length,
    RepeatedPtrField<CurveRange>* const out_range) {
  for (int i = 0; i < bound.boundary_type_size(); ++i) {
    if (!IsAllowedOut(bound.boundary_type(i))) {
      continue;
    }
    CurveRange* range = out_range->Add();
    range->mutable_start()->set_s(GetLengthbyRate(bound.boundary_type(i).s(),
                                                  bound.length(), lane_length));
    if (i != bound.boundary_type_size() - 1) {
      range->mutable_end()->set_s(GetLengthbyRate(
          bound.boundary_type(i + 1).s(), bound.length(), lane_length));
    } else {
      range->mutable_end()->set_s(lane_length);
    }
  }
}

void NodeCreator::InitNodeInfo(const Lane& lane, const std::string& road_id,
                               Node* const node,
                               const RoutingConfig* routingconfig) {
  double lane_length = GetLaneLength(lane);
  node->set_lane_id(lane.id().id());
  node->set_road_id(road_id);
  AddOutBoundary(lane.left_boundary(), lane_length, node->mutable_left_out());
  AddOutBoundary(lane.right_boundary(), lane_length, node->mutable_right_out());
  node->set_length(lane_length);
  node->mutable_central_curve()->CopyFrom(lane.central_curve());
  node->set_is_virtual(true);
  if (!lane.has_junction_id() ||
      lane.left_neighbor_forward_lane_id_size() > 0 ||
      lane.right_neighbor_forward_lane_id_size() > 0) {
    node->set_is_virtual(false);
  }
}

void NodeCreator::InitNodeCost(const Lane& lane, Node* const node,
                               const RoutingConfig* routingconfig) {
  double lane_length = GetLaneLength(lane);
  double speed_limit = (lane.has_speed_limit()) ? lane.speed_limit()
                                                : routingconfig->base_speed();
  double ratio = (speed_limit >= routingconfig->base_speed())
                     ? (1 / sqrt(speed_limit / routingconfig->base_speed()))
                     : 1.0;
  double cost = lane_length * ratio;
  if (lane.has_turn()) {
    if (lane.turn() == Lane::LEFT_TURN) {
      cost += routingconfig->left_turn_penalty();
    } else if (lane.turn() == Lane::RIGHT_TURN) {
      cost += routingconfig->right_turn_penalty();
    } else if (lane.turn() == Lane::U_TURN) {
      cost += routingconfig->uturn_penalty();
    }
  }
  node->set_cost(cost);
}

}  // namespace routing
}  // namespace EDrive
