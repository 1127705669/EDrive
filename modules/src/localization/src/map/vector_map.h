/******************************************************************************
 * Copyright 2024 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include "common/src/state.h"

#include <unordered_map>
#include <vector>
#include <string>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <tinyxml2.h>

/**
 * @namespace EDrive::localization
 * @brief EDrive::localization
 */
namespace EDrive {
namespace localization {

using EDrive::Result_state;
using namespace tinyxml2;

/**
 * @class VectorMap
 *
 * @brief VectorMap
 */
class VectorMap {
public:
  VectorMap() = default;
  ~VectorMap() = default;

  // 公共函数用于加载和解析OSM文件
  Result_state loadMap(const std::string &file, visualization_msgs::MarkerArray &marker_array);

private:
  struct Node {
    int id;
    double lat, lon, local_x, local_y, ele;

    Node(int id, double lat, double lon, double local_x, double local_y, double ele)
        : id(id), lat(lat), lon(lon), local_x(local_x), local_y(local_y), ele(ele) {}
  };

  struct Way {
    int id;
    std::vector<int> node_refs;

    Way(int id) : id(id) {}
  };

  struct Relation {
    int id;
    std::vector<int> left_refs;
    std::vector<int> right_refs;

    Relation(int id) : id(id) {}
  };

  void parse_osm(const std::string &file, std::unordered_map<int, Node> &nodes, std::unordered_map<int, Way> &ways, std::vector<Relation> &relations);
  void create_marker_array(const std::unordered_map<int, Node> &nodes, const std::unordered_map<int, Way> &ways, visualization_msgs::MarkerArray &marker_array);

  std::unordered_map<int, Node> nodes_;
  std::unordered_map<int, Way> ways_;
  std::vector<Relation> relations_;
};

} // namespace localization
} // namespace EDrive