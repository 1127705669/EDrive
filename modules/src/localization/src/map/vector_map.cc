/******************************************************************************
 * Copyright 2024 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#include "localization/src/map/vector_map.h"

namespace EDrive {
namespace localization {

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