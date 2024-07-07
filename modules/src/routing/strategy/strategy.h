/******************************************************************************
  * Copyright 2024 The EDrive Authors. All Rights Reserved.
  *****************************************************************************/

#pragma once

#include <unordered_set>
#include <vector>

#include "routing/graph/sub_topo_graph.h"
#include "routing/graph/topo_graph.h"
#include "routing/graph/topo_node.h"

namespace EDrive {
namespace routing {

class Strategy {
 public:
  virtual ~Strategy() {}

  virtual bool Search(const TopoGraph* graph, const SubTopoGraph* sub_graph,
                      const TopoNode* src_node, const TopoNode* dest_node,
                      std::vector<NodeWithRange>* const result_nodes) = 0;
};

}  // namespace routing
}  // namespace EDrive
