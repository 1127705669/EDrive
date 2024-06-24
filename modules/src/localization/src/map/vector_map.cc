/******************************************************************************
 * Copyright 2024 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#include "localization/src/map/vector_map.h"

namespace EDrive {
namespace localization {

using EDrive::common::Result_state;

Result_state VectorMap::loadMap(const std::string &file, visualization_msgs::MarkerArray &marker_array) {
    parse_osm(file, nodes_, ways_, relations_);
    create_marker_array(nodes_, ways_, marker_array);
    return Result_state::State_Ok;
}

void VectorMap::parse_osm(const std::string &file, std::unordered_map<int, Node> &nodes, std::unordered_map<int, Way> &ways, std::vector<Relation> &relations) {
    XMLDocument doc;
    if (doc.LoadFile(file.c_str()) != XML_SUCCESS) {
        ROS_ERROR("Failed to load file %s", file.c_str());
        return;
    }

    XMLElement *root = doc.RootElement();

    for (XMLElement *elem = root->FirstChildElement(); elem; elem = elem->NextSiblingElement()) {
        if (std::string(elem->Name()) == "node") {
            int id = std::stoi(elem->Attribute("id"));
            double lat = std::stod(elem->Attribute("lat"));
            double lon = std::stod(elem->Attribute("lon"));
            double local_x = 0, local_y = 0, ele = 0;

            for (XMLElement *tag = elem->FirstChildElement("tag"); tag; tag = tag->NextSiblingElement("tag")) {
                std::string key = tag->Attribute("k");
                if (key == "local_x") {
                    local_x = std::stod(tag->Attribute("v"));
                } else if (key == "local_y") {
                    local_y = std::stod(tag->Attribute("v"));
                } else if (key == "ele") {
                    ele = std::stod(tag->Attribute("v"));
                }
            }
            // Use emplace to construct the Node in-place
            nodes.emplace(std::piecewise_construct,
                          std::forward_as_tuple(id),
                          std::forward_as_tuple(id, lat, lon, local_x, local_y, ele));

        } else if (std::string(elem->Name()) == "way") {
            int id = std::stoi(elem->Attribute("id"));
            Way way(id);
            for (XMLElement *nd = elem->FirstChildElement("nd"); nd; nd = nd->NextSiblingElement("nd")) {
                way.node_refs.push_back(std::stoi(nd->Attribute("ref")));
            }
            ways.emplace(id, way);

        } else if (std::string(elem->Name()) == "relation") {
            int id = std::stoi(elem->Attribute("id"));
            Relation relation(id);
            for (XMLElement *member = elem->FirstChildElement("member"); member; member = member->NextSiblingElement("member")) {
                int ref = std::stoi(member->Attribute("ref"));
                std::string role = member->Attribute("role");
                if (role == "left") {
                    relation.left_refs.push_back(ref);
                } else if (role == "right") {
                    relation.right_refs.push_back(ref);
                }
            }
            relations.push_back(relation);
        }
    }
}

void VectorMap::publishMiddlePath(std::initializer_list<int> relation_ids, visualization_msgs::MarkerArray &path) {
    int marker_id = 0;
    for (int relation_id : relation_ids) {
        auto it = std::find_if(relations_.begin(), relations_.end(), [relation_id](const Relation& rel) {
            return rel.id == relation_id;
        });

        if (it == relations_.end()) {
            ROS_ERROR("Relation ID %d not found", relation_id);
            return;
        }

        const Relation& relation = *it;
        if (relation.left_refs.size() != relation.right_refs.size()) {
            ROS_WARN("Mismatch in sizes of left_refs and right_refs for relation ID %d", relation_id);
        }

        size_t min_size = std::min(relation.left_refs.size(), relation.right_refs.size());

        std::vector<geometry_msgs::Point> middle_points;
        for (size_t i = 0; i < min_size; ++i) {
            int left_way_id = relation.left_refs[i];
            int right_way_id = relation.right_refs[i];

            // 检查这两个way ID是否存在于ways_中
            if (ways_.find(left_way_id) == ways_.end()) {
                ROS_ERROR("Left way ID %d not found", left_way_id);
                continue;
            }
            if (ways_.find(right_way_id) == ways_.end()) {
                ROS_ERROR("Right way ID %d not found", right_way_id);
                continue;
            }

            const Way& left_way = ways_.at(left_way_id);
            const Way& right_way = ways_.at(right_way_id);

            // // 打印left_way的节点ID
            // std::cout << "Left Way ID " << left_way_id << " contains nodes: ";
            // for (int node_id : left_way.node_refs) {
            //     std::cout << node_id << " ";
            // }
            // std::cout << std::endl;

            // // 打印right_way的节点ID
            // std::cout << "Right Way ID " << right_way_id << " contains nodes: ";
            // for (int node_id : right_way.node_refs) {
            //     std::cout << node_id << " ";
            // }
            // std::cout << std::endl;

            size_t min_nodes_size = std::min(left_way.node_refs.size(), right_way.node_refs.size());
            for (size_t j = 0; j < min_nodes_size; ++j) {
                int left_node_id = left_way.node_refs[j];
                int right_node_id = right_way.node_refs[j];

                if (nodes_.find(left_node_id) == nodes_.end() || nodes_.find(right_node_id) == nodes_.end()) {
                    ROS_ERROR("Node ref %d or %d not found", left_node_id, right_node_id);
                    continue;
                }

                const Node& left_node = nodes_.at(left_node_id);
                const Node& right_node = nodes_.at(right_node_id);

                geometry_msgs::Point mid_point;
                mid_point.x = (left_node.local_x + right_node.local_x) / 2.0;
                mid_point.y = (left_node.local_y + right_node.local_y) / 2.0;
                mid_point.z = (left_node.ele + right_node.ele) / 2.0;

                visualization_msgs::Marker marker;
                marker.header.frame_id = "map";
                marker.header.stamp = ros::Time::now();
                marker.ns = "middle_points";
                marker.id = marker_id++;
                marker.type = visualization_msgs::Marker::SPHERE;
                marker.action = visualization_msgs::Marker::ADD;
                
                marker.pose.position.x = mid_point.x;
                marker.pose.position.y = mid_point.y;
                marker.pose.position.z = mid_point.z;

                marker.pose.orientation.x = 0.0;
                marker.pose.orientation.y = 0.0;
                marker.pose.orientation.z = 0.0;
                marker.pose.orientation.w = 1.0;

                marker.scale.x = 0.5;  // 设定 marker 的大小
                marker.scale.y = 0.5;
                marker.scale.z = 0.5;
                marker.color.a = 1.0; // 透明度
                marker.color.r = 0.0;
                marker.color.g = 1.0;
                marker.color.b = 0.0;

                path.markers.push_back(marker);
            }
        }
    }
    
}

void VectorMap::create_marker_array(const std::unordered_map<int, Node> &nodes, const std::unordered_map<int, Way> &ways, visualization_msgs::MarkerArray &marker_array) {
    int marker_id = 0;

    for (auto const& way_pair : ways) {
        int id = way_pair.first;
        const Way& way = way_pair.second;

        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "ways";
        marker.id = marker_id++;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.2;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;

        for (const int &ref : way.node_refs) {
            if (nodes.find(ref) != nodes.end()) {
                const Node &point = nodes.at(ref);
                geometry_msgs::Point p;
                p.x = point.local_x;
                p.y = point.local_y;
                p.z = point.ele;
                marker.points.push_back(p);
            }
        }
        marker_array.markers.push_back(marker);
    }
}

} // namespace localization
} // namespace EDrive