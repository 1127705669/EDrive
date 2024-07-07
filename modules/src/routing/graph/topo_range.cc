/******************************************************************************
 * Copyright 2024 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/


#include "routing/graph/topo_range.h"

#include <algorithm>

#include "routing/common/routing_gflags.h"

namespace EDrive {
namespace routing {

bool NodeSRange::IsEnoughForChangeLane(double start_s, double end_s) {
  return IsEnoughForChangeLane(end_s - start_s);
}

bool NodeSRange::IsEnoughForChangeLane(double length) {
  return (length > FLAGS_min_length_for_lane_change);
}

NodeSRange::NodeSRange(double s1, double s2) : start_s_(s1), end_s_(s2) {}

bool NodeSRange::operator<(const NodeSRange& other) const {
  return StartS() < other.StartS();
}

bool NodeSRange::IsValid() const { return start_s_ <= end_s_; }

double NodeSRange::StartS() const { return start_s_; }

double NodeSRange::EndS() const { return end_s_; }

double NodeSRange::Length() const { return end_s_ - start_s_; }

bool NodeSRange::IsEnoughForChangeLane() const {
  return NodeSRange::IsEnoughForChangeLane(StartS(), EndS());
}

void NodeSRange::SetStartS(double start_s) { start_s_ = start_s; }

void NodeSRange::SetEndS(double end_s) { end_s_ = end_s; }

bool NodeSRange::MergeRangeOverlap(const NodeSRange& other) {
  if (!IsValid() || !other.IsValid()) {
    return false;
  }
  if (other.StartS() > EndS() || other.EndS() < StartS()) {
    return false;
  }
  SetEndS(std::max(EndS(), other.EndS()));
  SetStartS(std::min(StartS(), other.StartS()));
  return true;
}

}  // namespace routing
}  // namespace EDrive
