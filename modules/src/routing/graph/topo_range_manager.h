/******************************************************************************
  * Copyright 2024 The EDrive Authors. All Rights Reserved.
  *****************************************************************************/

#pragma once

#include <unordered_map>
#include <vector>

#include "routing/graph/topo_node.h"
#include "routing/graph/topo_range.h"

namespace EDrive {
namespace routing {

class TopoRangeManager {
 public:
  TopoRangeManager() = default;
  virtual ~TopoRangeManager() = default;

  const std::unordered_map<const TopoNode*, std::vector<NodeSRange>>& RangeMap()
      const;
  bool Find(const TopoNode* node) const;
  double RangeStart(const TopoNode* node) const;
  double RangeEnd(const TopoNode* node) const;
  void PrintDebugInfo() const;

  void Clear();
  void Add(const TopoNode* node, double start_s, double end_s);
  void SortAndMerge();

 private:
  std::unordered_map<const TopoNode*, std::vector<NodeSRange>> range_map_;
};

}  // namespace routing
}  // namespace EDrive
