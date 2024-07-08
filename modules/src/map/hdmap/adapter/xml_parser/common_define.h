/******************************************************************************
 * Copyright 2024 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/
#pragma once

#include <string>
#include <unordered_set>
#include <vector>
#include "common/src/log.h"
#include "map/proto/map.pb.h"

namespace EDrive {
namespace hdmap {
namespace adapter {

using PbHeader = EDrive::hdmap::Header;
using PbRoad = EDrive::hdmap::Road;
using PbRoadSection = EDrive::hdmap::RoadSection;
using PbLane = EDrive::hdmap::Lane;
using PbJunction = EDrive::hdmap::Junction;
using PbSignal = EDrive::hdmap::Signal;
using PbSubSignal = EDrive::hdmap::Subsignal;
using PbCrosswalk = EDrive::hdmap::Crosswalk;
using PbParkingSpace = EDrive::hdmap::ParkingSpace;
using PbSpeedBump = EDrive::hdmap::SpeedBump;
using PbStopSign = EDrive::hdmap::StopSign;
using PbYieldSign = EDrive::hdmap::YieldSign;
using PbObjectOverlapInfo = EDrive::hdmap::ObjectOverlapInfo;
using PbOverlap = EDrive::hdmap::Overlap;
using PbClearArea = EDrive::hdmap::ClearArea;
using PbLineSegment = EDrive::hdmap::LineSegment;
using PbCurveSegment = EDrive::hdmap::CurveSegment;
using PbCurve = EDrive::hdmap::Curve;
using PbPoint3D = EDrive::common::PointENU;
using PbLaneType = EDrive::hdmap::Lane_LaneType;
using PbTurnType = EDrive::hdmap::Lane_LaneTurn;
using PbID = EDrive::hdmap::Id;
using PbLaneBoundary = EDrive::hdmap::LaneBoundary;
using PbLaneBoundaryTypeType = EDrive::hdmap::LaneBoundaryType_Type;
using PbPolygon = EDrive::hdmap::Polygon;
using PbBoundaryPolygon = EDrive::hdmap::BoundaryPolygon;
using PbBoundaryEdge = EDrive::hdmap::BoundaryEdge;

using PbLaneDirection = EDrive::hdmap::Lane_LaneDirection;
using PbSignalType = EDrive::hdmap::Signal_Type;
using PbSubSignalType = EDrive::hdmap::Subsignal_Type;
using PbBoundaryEdgeType = EDrive::hdmap::BoundaryEdge_Type;

struct StopLineInternal {
  std::string id;
  PbCurve curve;
};

struct StopSignInternal {
  std::string id;
  PbStopSign stop_sign;
  std::unordered_set<std::string> stop_line_ids;
};

struct YieldSignInternal {
  std::string id;
  PbYieldSign yield_sign;
  std::unordered_set<std::string> stop_line_ids;
};

struct TrafficLightInternal {
  std::string id;
  PbSignal traffic_light;
  std::unordered_set<std::string> stop_line_ids;
};

struct OverlapWithLane {
  std::string object_id;
  double start_s;
  double end_s;
  bool is_merge;

  OverlapWithLane() : is_merge(false) {}
};

struct OverlapWithJunction {
  std::string object_id;
};

struct LaneInternal {
  PbLane lane;
  std::vector<OverlapWithLane> overlap_signals;
  std::vector<OverlapWithLane> overlap_objects;
  std::vector<OverlapWithLane> overlap_junctions;
  std::vector<OverlapWithLane> overlap_lanes;
};

struct JunctionInternal {
  PbJunction junction;
  std::unordered_set<std::string> road_ids;
  std::vector<OverlapWithJunction> overlap_with_junctions;
};

struct RoadSectionInternal {
  std::string id;
  PbRoadSection section;
  std::vector<LaneInternal> lanes;
};

struct RoadInternal {
  std::string id;
  PbRoad road;

  bool in_junction;
  std::string junction_id;

  std::vector<RoadSectionInternal> sections;

  std::vector<TrafficLightInternal> traffic_lights;
  std::vector<StopSignInternal> stop_signs;
  std::vector<YieldSignInternal> yield_signs;
  std::vector<PbCrosswalk> crosswalks;
  std::vector<PbClearArea> clear_areas;
  std::vector<PbSpeedBump> speed_bumps;
  std::vector<StopLineInternal> stop_lines;
  std::vector<PbParkingSpace> parking_spaces;

  RoadInternal() : in_junction(false) { junction_id = ""; }
};

}  // namespace adapter
}  // namespace hdmap
}  // namespace EDrive
