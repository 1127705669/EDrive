/******************************************************************************
 * Copyright 2024 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#include "viewer/viewer_agent/perception_agent.h"

#include "common/src/log.h"

namespace EDrive {
namespace viewer {

using EDrive::common::Result_state;

PerceptionAgent::PerceptionAgent(const derived_object_msgs::ObjectArray& objects, visualization_msgs::MarkerArray& objects_marker_array)
 : objects_(objects), objects_marker_array_(objects_marker_array), name_("Perception Viewer Agent") {
  EINFO << "Using " << name_;
}

std::string PerceptionAgent::Name() const { return name_; }

PerceptionAgent::~PerceptionAgent() {}

Result_state PerceptionAgent::Init(const ViewerConf *viewer_conf) {
  return Result_state::State_Ok;
}

Result_state PerceptionAgent::ProcessData() {
  VisualizeObjects();
  return Result_state::State_Ok;
}

void PerceptionAgent::VisualizeObjects() {
    // 清空现有的标记
    objects_marker_array_.markers.clear();

    // 遍历所有对象
    for (const auto& obj : objects_.objects) {
        
        visualization_msgs::Marker marker;
        
        // 设置标记的命名空间和ID
        marker.ns = "object_markers";
        marker.id = obj.id;  // 假设每个对象有一个唯一的ID
        
        // 根据对象的形状类型设置标记类型和尺寸
        switch (obj.shape.type) {
            case shape_msgs::SolidPrimitive::BOX:
                marker.type = visualization_msgs::Marker::CUBE;
                if (obj.shape.dimensions[shape_msgs::SolidPrimitive::BOX_X] == 0) {
                    EERROR << "Invalid BOX_X dimension for object ID: " << obj.id;
                    continue;
                }
                if (obj.shape.dimensions[shape_msgs::SolidPrimitive::BOX_Y] == 0) {
                    EERROR << "Invalid BOX_Y dimension for object ID: " << obj.id;
                    continue;
                }
                if (obj.shape.dimensions[shape_msgs::SolidPrimitive::BOX_Z] == 0) {
                    EERROR << "Invalid BOX_Z dimension for object ID: " << obj.id;
                    continue;
                }
                marker.scale.x = obj.shape.dimensions[shape_msgs::SolidPrimitive::BOX_X];
                marker.scale.y = obj.shape.dimensions[shape_msgs::SolidPrimitive::BOX_Y];
                marker.scale.z = obj.shape.dimensions[shape_msgs::SolidPrimitive::BOX_Z];
                break;
            case shape_msgs::SolidPrimitive::SPHERE:
                marker.type = visualization_msgs::Marker::SPHERE;
                if (obj.shape.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS] == 0) {
                    EERROR << "Invalid SPHERE_RADIUS dimension for object ID: " << obj.id;
                    continue;
                }
                marker.scale.x = marker.scale.y = marker.scale.z = obj.shape.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS] * 2;  // 直径
                break;
            case shape_msgs::SolidPrimitive::CYLINDER:
                marker.type = visualization_msgs::Marker::CYLINDER;
                if (obj.shape.dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS] == 0) {
                    EERROR << "Invalid CYLINDER_RADIUS dimension for object ID: " << obj.id;
                    continue;
                }
                if (obj.shape.dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT] == 0) {
                    EERROR << "Invalid CYLINDER_HEIGHT dimension for object ID: " << obj.id;
                    continue;
                }
                marker.scale.x = marker.scale.y = obj.shape.dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS] * 2;  // 直径
                marker.scale.z = obj.shape.dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT];
                break;
            case shape_msgs::SolidPrimitive::CONE:
                marker.type = visualization_msgs::Marker::CYLINDER;  // RViz 不支持锥体，使用圆柱体替代
                if (obj.shape.dimensions[shape_msgs::SolidPrimitive::CONE_RADIUS] == 0) {
                    EERROR << "Invalid CONE_RADIUS dimension for object ID: " << obj.id;
                    continue;
                }
                if (obj.shape.dimensions[shape_msgs::SolidPrimitive::CONE_HEIGHT] == 0) {
                    EERROR << "Invalid CONE_HEIGHT dimension for object ID: " << obj.id;
                    continue;
                }
                marker.scale.x = marker.scale.y = obj.shape.dimensions[shape_msgs::SolidPrimitive::CONE_RADIUS] * 2;  // 底面直径
                marker.scale.z = obj.shape.dimensions[shape_msgs::SolidPrimitive::CONE_HEIGHT];
                break;
            default:
                EERROR << "Unknown shape type: " << obj.shape.type;
                continue;  // 跳过未知形状类型的对象
        }

        // 统一设置颜色为蓝色
        marker.color.r = 0.0f;
        marker.color.g = 0.0f;
        marker.color.b = 1.0f;
        marker.color.a = 1.0;

        // 设置动作：添加或更新标记
        marker.action = visualization_msgs::Marker::ADD;
        
        // 设置标记的位置和方向
        marker.pose.position.x = obj.pose.position.x;
        marker.pose.position.y = obj.pose.position.y;
        marker.pose.position.z = obj.pose.position.z;
        marker.pose.orientation.x = obj.pose.orientation.x;
        marker.pose.orientation.y = obj.pose.orientation.y;
        marker.pose.orientation.z = obj.pose.orientation.z;
        marker.pose.orientation.w = obj.pose.orientation.w;
        
        // 设置标记的持续时间（0表示永久）
        marker.lifetime = ros::Duration();

        // 设置 frame_id
        marker.header.frame_id = "map"; // 或者其他适合你的应用的坐标系名称
        marker.header.stamp = ros::Time::now(); // 设置当前时间戳

        // 添加标记到数组
        objects_marker_array_.markers.push_back(marker);
    }
}





} // namespace viewer
} // namespace EDrive
