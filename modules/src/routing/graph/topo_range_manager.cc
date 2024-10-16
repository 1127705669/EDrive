/******************************************************************************
  * Copyright 2024 The EDrive Authors. All Rights Reserved.
  *****************************************************************************/

#include "routing/graph/topo_range_manager.h"

#include <algorithm>
#include <utility>

#include "common/src/log.h"
#include "common/util/map_util.h"

namespace EDrive {
namespace routing {
namespace {

using EDrive::common::util::ContainsKey;
using EDrive::common::util::FindOrDieNoPrint;

void merge_block_range(const TopoNode* topo_node,
                       const std::vector<NodeSRange>& origin_range,
                       std::vector<NodeSRange>* block_range) {
  std::vector<NodeSRange> sorted_origin_range;
  sorted_origin_range.insert(sorted_origin_range.end(), origin_range.begin(),
                             origin_range.end());
  sort(sorted_origin_range.begin(), sorted_origin_range.end());
  int cur_index = 0;
  int total_size = sorted_origin_range.size();
  while (cur_index < total_size) {
    NodeSRange range(sorted_origin_range[cur_index]);
    ++cur_index;
    while (cur_index < total_size &&
           range.MergeRangeOverlap(sorted_origin_range[cur_index])) {
      ++cur_index;
    }
    if (range.EndS() < topo_node->StartS() ||
        range.StartS() > topo_node->EndS()) {
      continue;
    }
    range.SetStartS(std::max(topo_node->StartS(), range.StartS()));
    range.SetEndS(std::min(topo_node->EndS(), range.EndS()));
    block_range->push_back(std::move(range));
  }
}

}  // namespace

const std::unordered_map<const TopoNode*, std::vector<NodeSRange>>&
TopoRangeManager::RangeMap() const {
  return range_map_;
}

bool TopoRangeManager::Find(const TopoNode* node) const {
  return ContainsKey(range_map_, node);
}

double TopoRangeManager::RangeStart(const TopoNode* node) const {
  const auto& range = FindOrDieNoPrint(range_map_, node);
  return range.front().StartS();
}

double TopoRangeManager::RangeEnd(const TopoNode* node) const {
  const auto& range = FindOrDieNoPrint(range_map_, node);
  return range.back().EndS();
}

void TopoRangeManager::PrintDebugInfo() const {
  for (const auto& map : range_map_) {
    for (const auto& range : map.second) {
      EINFO << "black lane id: " << map.first->LaneId()
            << ", start s: " << range.StartS() << ", end s: " << range.EndS();
    }
  }
}

void TopoRangeManager::Clear() { range_map_.clear(); }

void TopoRangeManager::Add(const TopoNode* node, double start_s, double end_s) {
  NodeSRange range(start_s, end_s);
  range_map_[node].push_back(range);
}

void TopoRangeManager::SortAndMerge() {
  for (auto& iter : range_map_) {
    std::vector<NodeSRange> merged_range_vec;
    merge_block_range(iter.first, iter.second, &merged_range_vec);
    iter.second.clear();
    iter.second.insert(iter.second.begin(), merged_range_vec.begin(),
                       merged_range_vec.end());
  }
}

}  // namespace routing
}  // namespace EDrive
