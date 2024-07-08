/******************************************************************************
  * Copyright 2024 The EDrive Authors. All Rights Reserved.
  *****************************************************************************/

#pragma once

#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "routing/strategy/strategy.h"

namespace EDrive {
namespace routing {

class AStarStrategy : public Strategy {
 public:
  explicit AStarStrategy(bool enable_change);
  ~AStarStrategy() = default;

  virtual bool Search(const TopoGraph* graph, const SubTopoGraph* sub_graph,
                      const TopoNode* src_node, const TopoNode* dest_node,
                      std::vector<NodeWithRange>* const result_nodes);

 private:
  void Clear();
  double HeuristicCost(const TopoNode* src_node, const TopoNode* dest_node);
  double GetResidualS(const TopoNode* node);
  double GetResidualS(const TopoEdge* edge, const TopoNode* to_node);

 private:
  bool change_lane_enabled_;
  std::unordered_set<const TopoNode*> open_set_;
  std::unordered_set<const TopoNode*> closed_set_;
  std::unordered_map<const TopoNode*, const TopoNode*> came_from_;
  std::unordered_map<const TopoNode*, double> g_score_;
  std::unordered_map<const TopoNode*, double> enter_s_;
};

}  // namespace routing
}  // namespace EDrive
