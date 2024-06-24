/******************************************************************************
 * Copyright 2022 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#include "perception/src/perception.h"

#include "common/adapters/adapter_manager.h"

#include "common/util/file.h"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

namespace EDrive {
namespace perception {

using EDrive::common::Result_state;
using EDrive::common::adapter::AdapterManager;

std::string Perception::Name() const { return "EDrive_perception"; }

Result_state Perception::Init(){
  ROS_INFO("Perception init, starting...");

  root_path = EDrive::common::util::GetRootPath();
  adapter_conf_file = root_path + adapter_conf_file;
  perception_conf_file = root_path + perception_conf_file;

  ROS_INFO("  registering node: %s", Name().c_str());
  AdapterManager::Init(adapter_conf_file);

  if (!AdapterManager::Initialized()) {
    ROS_INFO("  registering node: %s", Name().c_str());
    AdapterManager::Init(adapter_conf_file);
  }

  return Result_state::State_Ok;
}

Result_state Perception::CheckInput() {
  AdapterManager::Observe();

  auto objects_adapter = AdapterManager::GetCarlaObjects();
  Carla_objects_ = objects_adapter->GetLatestObserved();

  auto road_markings_adapter = AdapterManager::GetRoadMarkings();
  Carla_image_ = road_markings_adapter->GetLatestObserved();

  ConvertImageToVisualization(Carla_image_);

  return Result_state::State_Ok;
}

Result_state Perception::ConvertImageToVisualization(sensor_msgs::Image& image_msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return Result_state::State_Failed;
    }

    // 在窗口中显示图像
    // cv::imshow("Semantic Segmentation Image", cv_ptr->image);
    // cv::waitKey(1);  // 等待1毫秒，让OpenCV有时间渲染图像

    // 清除之前的点
    line_strip_.points.clear();

    // 创建 Marker
    line_strip_.header.frame_id = "map";
    line_strip_.header.stamp = ros::Time::now();
    line_strip_.ns = "road_boundaries";
    line_strip_.action = visualization_msgs::Marker::ADD;
    line_strip_.pose.orientation.w = 1.0;
    line_strip_.id = 0;
    line_strip_.type = visualization_msgs::Marker::LINE_STRIP;
    line_strip_.scale.x = 0.05;  // 线宽
    line_strip_.color.r = 0.196;  // 道路标线颜色 RGB (50, 234, 157) 转换为0-1范围
    line_strip_.color.g = 0.917;
    line_strip_.color.b = 0.196;
    line_strip_.color.a = 1.0;  // 不透明度

    // 识别车道线颜色并添加点到 Marker
    for (int y = 0; y < cv_ptr->image.rows; y++) {
        for (int x = 0; x < cv_ptr->image.cols; x++) {
            cv::Vec3b color = cv_ptr->image.at<cv::Vec3b>(y, x);
            if (color[0] == 157 && color[1] == 234 && color[2] == 50) {  // BGR顺序
                geometry_msgs::Point p;
                p.x = x;  // 需要根据实际情况转换为适当的坐标
                p.y = y;  // 需要根据实际情况转换为适当的坐标
                p.z = 0;
                line_strip_.points.push_back(p);
            }
        }
    }

    if (line_strip_.points.empty()) {
        ROS_WARN("No points added to the marker.");
    }

    return Result_state::State_Ok;
}

Result_state Perception::Start(){

  ROS_INFO("Perception resetting vehicle state, sleeping for 1000 ms ...");
  ros::Duration(1.0).sleep();

  timer_ = common::adapter::AdapterManager::CreateTimer(ros::Duration(control_perception_.perception_period()), 
                                                                &Perception::OnTimer,
                                                                this);

  ROS_INFO("Perception init done!");
  ROS_INFO("Perception started");
  
  return Result_state::State_Ok;
}

void Perception::OnTimer(const ros::TimerEvent &) {
  Result_state state = CheckInput();
  objects_ = Carla_objects_;
  Publish();
}

void Perception::Publish(){
  AdapterManager::PublishPerception(objects_);
  AdapterManager::PublishRoadMarker(line_strip_);
}

void Perception::Stop() {
  
}

} // namespace perception
} // namespace EDrive