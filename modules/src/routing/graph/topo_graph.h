/******************************************************************************
  * Copyright 2024 The EDrive Authors. All Rights Reserved.
  *****************************************************************************/

#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "common/src/log.h"
#include "routing/graph/topo_node.h"
#include "routing/proto/topo_graph.pb.h"

namespace EDrive {
namespace routing {

class TopoGraph {
 public:
  TopoGraph() = default;
  ~TopoGraph() = default;

  bool LoadGraph(const Graph& filename);

  const std::string& MapVersion() const;
  const std::string& MapDistrict() const;
  const TopoNode* GetNode(const std::string& id) const;
  void GetNodesByRoadId(
      const std::string& road_id,
      std::unordered_set<const TopoNode*>* const node_in_road) const;

 private:
  void Clear();
  bool LoadNodes(const Graph& graph);
  bool LoadEdges(const Graph& graph);

 private:
  std::string map_version_;
  std::string map_district_;
  std::vector<std::shared_ptr<TopoNode> > topo_nodes_;
  std::vector<std::shared_ptr<TopoEdge> > topo_edges_;
  std::unordered_map<std::string, int> node_index_map_;
  std::unordered_map<std::string, std::unordered_set<const TopoNode*> >
      road_node_map_;
};

}  // namespace routing
}  // namespace EDrive
