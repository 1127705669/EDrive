/******************************************************************************
  * Copyright 2024 The EDrive Authors. All Rights Reserved.
  *****************************************************************************/

#include "routing/graph/topo_graph.h"

#include <utility>

#include "common/util/file.h"
#include "routing/graph/topo_node.h"

namespace EDrive {
namespace routing {

void TopoGraph::Clear() {
  topo_nodes_.clear();
  topo_edges_.clear();
  node_index_map_.clear();
}

bool TopoGraph::LoadNodes(const Graph& graph) {
  if (graph.node_size() == 0) {
    EERROR << "No nodes found in topology graph.";
    return false;
  }
  for (const auto& node : graph.node()) {
    node_index_map_[node.lane_id()] = topo_nodes_.size();
    std::shared_ptr<TopoNode> topo_node;
    topo_node.reset(new TopoNode(node));
    road_node_map_[node.road_id()].insert(topo_node.get());
    topo_nodes_.push_back(std::move(topo_node));
  }
  return true;
}

// Need to execute load_nodes() firstly
bool TopoGraph::LoadEdges(const Graph& graph) {
  if (graph.edge_size() == 0) {
    EINFO << "0 edges found in topology graph, but it's fine";
    return true;
  }
  for (const auto& edge : graph.edge()) {
    const std::string& from_lane_id = edge.from_lane_id();
    const std::string& to_lane_id = edge.to_lane_id();
    if (node_index_map_.count(from_lane_id) != 1 ||
        node_index_map_.count(to_lane_id) != 1) {
      return false;
    }
    std::shared_ptr<TopoEdge> topo_edge;
    TopoNode* from_node = topo_nodes_[node_index_map_[from_lane_id]].get();
    TopoNode* to_node = topo_nodes_[node_index_map_[to_lane_id]].get();
    topo_edge.reset(new TopoEdge(edge, from_node, to_node));
    from_node->AddOutEdge(topo_edge.get());
    to_node->AddInEdge(topo_edge.get());
    topo_edges_.push_back(std::move(topo_edge));
  }
  return true;
}

bool TopoGraph::LoadGraph(const Graph& graph) {
  Clear();

  map_version_ = graph.hdmap_version();
  map_district_ = graph.hdmap_district();

  if (!LoadNodes(graph)) {
    EERROR << "Failed to load nodes from topology graph.";
    return false;
  }
  if (!LoadEdges(graph)) {
    EERROR << "Failed to load edges from topology graph.";
    return false;
  }
  EINFO << "Load Topo data succesful.";
  return true;
}

const std::string& TopoGraph::MapVersion() const { return map_version_; }

const std::string& TopoGraph::MapDistrict() const { return map_district_; }

const TopoNode* TopoGraph::GetNode(const std::string& id) const {
  const auto& iter = node_index_map_.find(id);
  if (iter == node_index_map_.end()) {
    return nullptr;
  }
  return topo_nodes_[iter->second].get();
}

void TopoGraph::GetNodesByRoadId(
    const std::string& road_id,
    std::unordered_set<const TopoNode*>* const node_in_road) const {
  const auto& iter = road_node_map_.find(road_id);
  if (iter != road_node_map_.end()) {
    node_in_road->insert(iter->second.begin(), iter->second.end());
  }
}

}  // namespace routing
}  // namespace EDrive
