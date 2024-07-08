/******************************************************************************
  * Copyright 2024 The EDrive Authors. All Rights Reserved.
  *****************************************************************************/

#pragma once

#include <string>
#include <utility>
#include <vector>

#include "routing/proto/routing.pb.h"

#include "routing/graph/node_with_range.h"
#include "routing/graph/topo_graph.h"
#include "routing/graph/topo_range_manager.h"

namespace EDrive {
namespace routing {

class ResultGenerator {
 public:
  ResultGenerator() = default;
  ~ResultGenerator() = default;

  bool GeneratePassageRegion(const std::string& map_version,
                             const RoutingRequest& request,
                             const std::vector<NodeWithRange>& nodes,
                             const TopoRangeManager& range_manager,
                             RoutingResponse* const result);

 private:
  struct PassageInfo {
    std::vector<NodeWithRange> nodes;
    ChangeLaneType change_lane_type;
    PassageInfo() = default;
    PassageInfo(const std::vector<NodeWithRange>& _nodes,
                ChangeLaneType _change_lane_type)
        : nodes(_nodes), change_lane_type(_change_lane_type) {}
  };

  bool GeneratePassageRegion(const std::vector<NodeWithRange>& nodes,
                             const TopoRangeManager& range_manager,
                             RoutingResponse* const result);

  void CreateRoadSegments(const std::vector<PassageInfo>& passages,
                          RoutingResponse* result);

  void AddRoadSegment(const std::vector<PassageInfo>& passages,
                      const std::pair<std::size_t, std::size_t>& start,
                      const std::pair<std::size_t, std::size_t>& end,
                      RoutingResponse* result);
  void ExtendPassages(const TopoRangeManager& range_manager,
                      std::vector<PassageInfo>* const passages);
  bool ExtractBasicPassages(const std::vector<NodeWithRange>& nodes,
                            std::vector<PassageInfo>* const passsages);
  void ExtendBackward(const TopoRangeManager& range_manager,
                      const PassageInfo& prev_passage,
                      PassageInfo* const curr_passage);
  void ExtendForward(const TopoRangeManager& range_manager,
                     const PassageInfo& next_passage,
                     PassageInfo* const curr_passage);
  bool IsReachableFromWithChangeLane(const TopoNode* from_node,
                                     const PassageInfo& to_nodes,
                                     NodeWithRange* reachable_node);
  bool IsReachableToWithChangeLane(const TopoNode* from_node,
                                   const PassageInfo& to_nodes,
                                   NodeWithRange* reachable_node);
};

}  // namespace routing
}  // namespace EDrive
