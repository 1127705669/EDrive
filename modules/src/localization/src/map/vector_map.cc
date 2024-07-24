/******************************************************************************
 * Copyright 2024 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#include "localization/src/map/vector_map.h"

#include <cmath> 

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

std::vector<VectorMap::RoadSegment> VectorMap::sortSegmentsByProximity(std::vector<RoadSegment>& segments) {
    if (segments.empty()) return {};

    std::vector<RoadSegment> sortedSegments;
    std::unordered_set<int> includedIndices; // 用于跟踪已经包括在排序中的段

    // 从第一个段开始
    sortedSegments.push_back(segments[0]);
    includedIndices.insert(0);

    while (sortedSegments.size() < segments.size()) {
        auto& lastSegment = sortedSegments.back();
        double minDistance = std::numeric_limits<double>::max();
        int nextIndex = -1;

        // 找出与最后一个段的最后一个点最近的段的第一个点
        for (int i = 0; i < segments.size(); ++i) {
            if (includedIndices.find(i) != includedIndices.end()) continue; // 跳过已经包括的段

            double distance = calculateDistance(lastSegment.points.back(), segments[i].points.front());
            if (distance < minDistance) {
                minDistance = distance;
                nextIndex = i;
            }
        }

        if (nextIndex != -1) {
            sortedSegments.push_back(segments[nextIndex]);
            includedIndices.insert(nextIndex);
        } else {
            // 如果没有找到合适的下一个段，可能是因为某些段是孤立的，这种情况下可以考虑直接添加剩余的段
            for (int i = 0; i < segments.size(); ++i) {
                if (includedIndices.find(i) == includedIndices.end()) {
                    sortedSegments.push_back(segments[i]);
                    includedIndices.insert(i);
                }
            }
        }
    }

    return sortedSegments;
}

double VectorMap::calculateDistance(const geometry_msgs::Point& a, const geometry_msgs::Point& b) {
    return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2) + pow(a.z - b.z, 2));
}

double VectorMap::calculateTheta(const geometry_msgs::Point& current, const geometry_msgs::Point& previous) {
    double dx = current.x - previous.x;
    double dy = current.y - previous.y;
    return atan2(dy, dx);
}

double VectorMap::calculateKappa(const geometry_msgs::Point& prev, const geometry_msgs::Point& current, const geometry_msgs::Point& next) {
    double dx1 = current.x - prev.x;
    double dy1 = current.y - prev.y;
    double dx2 = next.x - current.x;
    double dy2 = next.y - current.y;
    
    double numerator = fabs(dx1 * dy2 - dy1 * dx2);
    double denominator = pow(dx1 * dx1 + dy1 * dy1, 1.5) + pow(dx2 * dx2 + dy2 * dy2, 1.5);
    return denominator != 0 ? numerator / denominator : 0.0;
}

void VectorMap::publishMiddlePath(std::initializer_list<int> relation_ids, visualization_msgs::MarkerArray &path, ::planning::ADCTrajectory& trajectory_pb) {
    int marker_id = 0;
    std::vector<RoadSegment> allSegments;

    // 收集所有道路段的数据
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
            continue;
        }

        size_t min_size = std::min(relation.left_refs.size(), relation.right_refs.size());
        std::vector<geometry_msgs::Point> points;

        for (size_t i = 0; i < min_size; ++i) {
            int left_way_id = relation.left_refs[i];
            int right_way_id = relation.right_refs[i];

            if (ways_.find(left_way_id) == ways_.end() || ways_.find(right_way_id) == ways_.end()) {
                continue;
            }

            const Way& left_way = ways_.at(left_way_id);
            const Way& right_way = ways_.at(right_way_id);
            size_t min_nodes_size = std::min(left_way.node_refs.size(), right_way.node_refs.size());

            for (size_t j = 0; j < min_nodes_size; ++j) {
                const Node& left_node = nodes_.at(left_way.node_refs[j]);
                const Node& right_node = nodes_.at(right_way.node_refs[j]);
                geometry_msgs::Point mid_point;
                mid_point.x = (left_node.local_x + right_node.local_x) / 2.0;
                mid_point.y = (left_node.local_y + right_node.local_y) / 2.0;
                mid_point.z = (left_node.ele + right_node.ele) / 2.0;

                points.push_back(mid_point);
            }
        }

        allSegments.push_back({points, relation_id});
    }

    // 假设第一个道路段已经正确处理
    std::vector<RoadSegment> sortedSegments = sortSegmentsByProximity(allSegments);

    // 使用封闭路径的特性
    double initialTheta = 0.0;  // 用于存储路径的初始 Theta 值
    double initialKappa = 0.0;  // 用于存储路径的初始 Kappa 值
    geometry_msgs::Point firstPoint, lastPoint;

    // 计算初始Theta的条件检查
    if (!sortedSegments.empty() && !sortedSegments.front().points.empty()) {
        firstPoint = sortedSegments.front().points.front();
        if (!sortedSegments.back().points.empty()) {
            lastPoint = sortedSegments.back().points.back();
            if (firstPoint.x == lastPoint.x && firstPoint.y == lastPoint.y) {
                // 如果第一个点和最后一个点相同，则使用第二个点（如果存在）
                if (sortedSegments.front().points.size() > 1) {
                    initialTheta = calculateTheta(sortedSegments.front().points[1], firstPoint);
                }
            } else {
                initialTheta = calculateTheta(firstPoint, lastPoint);
            }
        }
    }

    // 遍历排序后的所有道路段
    for (int i = 0; i < sortedSegments.size(); ++i) {
        auto& segment = sortedSegments[i];
        for (size_t j = 0; j < segment.points.size(); ++j) {
            ::common::TrajectoryPoint tp;
            tp.path_point.x = segment.points[j].x;
            tp.path_point.y = segment.points[j].y;
            tp.path_point.z = segment.points[j].z;

            // Theta calculation
            if (j > 0) {
                tp.path_point.theta = calculateTheta(segment.points[j], segment.points[j - 1]);
            } else if (i > 0) {
                tp.path_point.theta = calculateTheta(segment.points[j], sortedSegments[i - 1].points.back());
            } else {
                tp.path_point.theta = initialTheta;
            }

            // Kappa calculation
            if (j > 0 && j < segment.points.size() - 1) {
                tp.path_point.kappa = calculateKappa(segment.points[j - 1], segment.points[j], segment.points[j + 1]);
            } else if (j == 0 && segment.points.size() > 1) {
                // 对于第一个点，如果是段的第一个点，使用前一段的最后一个点和当前段的下一个点
                if (i > 0 && sortedSegments[i - 1].points.size() > 0) {
                    tp.path_point.kappa = calculateKappa(sortedSegments[i - 1].points.back(), segment.points[j], segment.points[j + 1]);
                } else {
                    // 封闭路径的第一个段的第一个点
                    tp.path_point.kappa = calculateKappa(sortedSegments.back().points.back(), segment.points[j], segment.points[j + 1]);
                }
            } else if (j == segment.points.size() - 1 && i < sortedSegments.size() - 1 && sortedSegments[i + 1].points.size() > 0) {
                // 对于最后一个点，使用当前段的前一个点和下一段的第一个点
                tp.path_point.kappa = calculateKappa(segment.points[j - 1], segment.points[j], sortedSegments[i + 1].points.front());
            } else {
                // 封闭路径的最后一个段的最后一个点
                tp.path_point.kappa = calculateKappa(segment.points[j - 1], segment.points[j], sortedSegments.front().points.front());
            }

            trajectory_pb.trajectory_point.push_back(tp);
        }
    }


    int idx = 0;  // 添加一个索引来跟踪每个元素的序列号
    for (::common::TrajectoryPoint j : trajectory_pb.trajectory_point) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "middle_points";
        marker.id = marker_id++;
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;  // 设置marker类型为文本
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = j.path_point.x;
        marker.pose.position.y = j.path_point.y;
        marker.pose.position.z = j.path_point.z;

        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        // 文本标记只需要设定scale.z，代表文本的高度
        marker.scale.z = 0.5;  // 设置文本的高度

        // 设置marker的文本内容为当前的 idx 值
        std::stringstream ss;
        ss << idx;
        marker.text = ss.str();  // 将 idx 转换为字符串并设置为文本内容

        // 为文本设置颜色和透明度
        marker.color.a = 1.0;  // Alpha 必须设为1才能看见
        if (idx == 0) {
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
        } else if (idx == trajectory_pb.trajectory_point.size() - 1) {
            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 1.0;
        } else {
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
        }

        path.markers.push_back(marker);
        idx++;  // 每次循环后递增索引
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
        marker.color.r = 1.0;
        marker.color.g = 1.0;
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