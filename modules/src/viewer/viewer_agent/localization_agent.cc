/******************************************************************************
 * Copyright 2024 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#include "viewer/viewer_agent/localization_agent.h"

#include "common/src/log.h"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "common/configs/vehicle_config_helper.h"

namespace EDrive {
namespace viewer {

using EDrive::common::Result_state;
using EDrive::common::VehicleConfigHelper;

LocalizationAgent::LocalizationAgent(const nav_msgs::Odometry& ego_vehicle_localization, visualization_msgs::Marker& ego_vehicle_marker)
 : ego_vehicle_odometry_(ego_vehicle_localization), ego_vehicle_marker_(ego_vehicle_marker), name_("Localization Viewer Agent") {
  EINFO << "Using " << name_;
}

std::string LocalizationAgent::Name() const { return name_; }

LocalizationAgent::~LocalizationAgent() {}

Result_state LocalizationAgent::Init(const ViewerConf *viewer_conf) {
  vehicle_param_ = VehicleConfigHelper::Instance()->GetConfig().vehicle_param();
  return Result_state::State_Ok;
}

Result_state LocalizationAgent::ProcessData() {
  VisualizeLocalization();
  return Result_state::State_Ok;
}

void LocalizationAgent::VisualizeLocalization() {
    // 设置 frame ID 和时间戳
    ego_vehicle_marker_.header.frame_id = "map";
    ego_vehicle_marker_.header.stamp = ros::Time::now();

    // 设置命名空间和ID，这个ID应该在同一个命名空间中是唯一的
    ego_vehicle_marker_.ns = "vehicle_shape";
    ego_vehicle_marker_.id = 0;

    // 设置 marker 类型为线列表
    ego_vehicle_marker_.type = visualization_msgs::Marker::LINE_LIST;

    // 设置 marker 动作，ADD 为添加，也可用 MODIFY 或 DELETE
    ego_vehicle_marker_.action = visualization_msgs::Marker::ADD;

    // 设置 marker 的尺寸（线宽）
    ego_vehicle_marker_.scale.x = 0.1;  // 设置线宽

    // 设置颜色和透明度（绿色）
    ego_vehicle_marker_.color.r = 0.0;
    ego_vehicle_marker_.color.g = 1.0;
    ego_vehicle_marker_.color.b = 0.0;
    ego_vehicle_marker_.color.a = 1.0; // 不透明

    // 定义立方体的八个顶点（假设车的长宽高为4.90m x 1.93m x 1.62m）
    geometry_msgs::Point p[8];
    double lx = vehicle_param_.length() / 2.0;
    double ly = vehicle_param_.width() / 2.0;
    double lz = vehicle_param_.height();

    // 顶点在局部坐标系下的位置，底部与地面齐平
    p[0].x = -lx; p[0].y = -ly; p[0].z = 0.0;
    p[1].x = lx; p[1].y = -ly; p[1].z = 0.0;
    p[2].x = lx; p[2].y = ly; p[2].z = 0.0;
    p[3].x = -lx; p[3].y = ly; p[3].z = 0.0;
    p[4].x = -lx; p[4].y = -ly; p[4].z = lz;
    p[5].x = lx; p[5].y = -ly; p[5].z = lz;
    p[6].x = lx; p[6].y = ly; p[6].z = lz;
    p[7].x = -lx; p[7].y = ly; p[7].z = lz;

    // 获取车辆的姿态四元数
    tf2::Quaternion quat(
        ego_vehicle_odometry_.pose.pose.orientation.x,
        ego_vehicle_odometry_.pose.pose.orientation.y,
        ego_vehicle_odometry_.pose.pose.orientation.z,
        ego_vehicle_odometry_.pose.pose.orientation.w
    );

    // 将局部坐标系下的顶点转换为全局坐标系
    for (int i = 0; i < 8; ++i) {
        tf2::Vector3 point(p[i].x, p[i].y, p[i].z);
        tf2::Vector3 transformed_point = tf2::quatRotate(quat, point);
        p[i].x = ego_vehicle_odometry_.pose.pose.position.x + transformed_point.x();
        p[i].y = ego_vehicle_odometry_.pose.pose.position.y + transformed_point.y();
        p[i].z = ego_vehicle_odometry_.pose.pose.position.z + transformed_point.z();
    }

    // 添加立方体的十二条棱线
    ego_vehicle_marker_.points.clear();
    ego_vehicle_marker_.points.push_back(p[0]); ego_vehicle_marker_.points.push_back(p[1]);
    ego_vehicle_marker_.points.push_back(p[1]); ego_vehicle_marker_.points.push_back(p[2]);
    ego_vehicle_marker_.points.push_back(p[2]); ego_vehicle_marker_.points.push_back(p[3]);
    ego_vehicle_marker_.points.push_back(p[3]); ego_vehicle_marker_.points.push_back(p[0]);
    ego_vehicle_marker_.points.push_back(p[4]); ego_vehicle_marker_.points.push_back(p[5]);
    ego_vehicle_marker_.points.push_back(p[5]); ego_vehicle_marker_.points.push_back(p[6]);
    ego_vehicle_marker_.points.push_back(p[6]); ego_vehicle_marker_.points.push_back(p[7]);
    ego_vehicle_marker_.points.push_back(p[7]); ego_vehicle_marker_.points.push_back(p[4]);
    ego_vehicle_marker_.points.push_back(p[0]); ego_vehicle_marker_.points.push_back(p[4]);
    ego_vehicle_marker_.points.push_back(p[1]); ego_vehicle_marker_.points.push_back(p[5]);
    ego_vehicle_marker_.points.push_back(p[2]); ego_vehicle_marker_.points.push_back(p[6]);
    ego_vehicle_marker_.points.push_back(p[3]); ego_vehicle_marker_.points.push_back(p[7]);
}

} // namespace viewer
} // namespace EDrive
