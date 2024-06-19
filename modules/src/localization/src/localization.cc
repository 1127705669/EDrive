/******************************************************************************
 * Copyright 2022 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#include <ros/ros.h>
#include "localization/src/localization.h"

#include "localization/src/common/localization_gflags.h"
#include "common/src/util/file.h"
#include "common/src/adapters/adapter_manager.h"

namespace EDrive {
namespace localization {

using EDrive::common::Result_state;
using EDrive::common::adapter::AdapterManager;
  
std::string Localization::Name() const { return "EDrive_localization"; }

Result_state Localization::Init(){

  ROS_INFO("Localization init, starting...");

  root_path = EDrive::common::util::GetRootPath();
  adapter_conf_file = root_path + adapter_conf_file;
  localization_conf_file = root_path + localization_conf_file;

  ROS_INFO("  registering node: %s", Name().c_str());
  AdapterManager::Init(adapter_conf_file);

  EDrive::common::util::GetProtoFromASIIFile(localization_conf_file, &localization_conf_);
  
  /*
    * Example usage of PCDConvert function:
    * 
    * 1. Apply filtering and save the filtered point cloud:
    *    localization.PCDConvert(input_file, true, true, output_file);
    *    - input_file: Path to the input PCD file
    *    - true: Apply filtering (true indicates that filtering should be applied)
    *    - true: Save the filtered point cloud (true indicates that the filtered point cloud should be saved)
    *    - output_file: Path to the output filtered PCD file
    * 
    * 2. Apply filtering but do not save the filtered point cloud:
    *    localization.PCDConvert(input_file, true, false);
    *    - input_file: Path to the input PCD file
    *    - true: Apply filtering (true indicates that filtering should be applied)
    *    - false: Do not save the filtered point cloud (false indicates that the filtered point cloud should not be saved)
    * 
    * 3. Do not apply filtering, use the original point cloud:
    *    localization.PCDConvert(input_file, false, false);
    *    - input_file: Path to the input PCD file
    *    - false: Do not apply filtering (false indicates that filtering should not be applied)
    *    - false: Do not save the filtered point cloud (this parameter is irrelevant when filtering is not applied)
    */
  PCDConvert(root_path + cloud_point_map_conf_file, false, false);

  // loadAndPublishLanelet2Map(root_path + vector_map_conf_file);
  auto retValue = vector_mapper_.loadMap(root_path + vector_map_conf_file, vector_map_);

  AdapterManager::PublishCloudPointMap(cloud_point_map_);
  AdapterManager::PublishVectorMap(vector_map_);

  return Result_state::State_Ok;
}

Result_state Localization::CheckInput() {
  AdapterManager::Observe();
  auto position_adapter = AdapterManager::GetVehicle();
  position_odometry_ = position_adapter->GetLatestObserved();

  return Result_state::State_Ok;
}

Result_state Localization::Start(){

  timer_ = common::adapter::AdapterManager::CreateTimer(ros::Duration(localization_conf_.localization_period()), 
                                                                &Localization::OnTimer,
                                                                this);
  ROS_INFO("Localization init done!");
  ROS_INFO("Localization started");
  
  return Result_state::State_Ok;
}

void Localization::OnTimer(const ros::TimerEvent &) {
  Result_state status = CheckInput();
  PositionConvert();

  Publish();
}

void Localization::PositionConvert() {

    // 设置 frame ID 和时间戳
    position_marker_.header.frame_id = "map";
    position_marker_.header.stamp = ros::Time::now();
    
    // 设置命名空间和ID，这个ID应该在同一个命名空间中是唯一的
    position_marker_.ns = "vehicle_shape";
    position_marker_.id = 0;
    
    // 设置 marker 类型为立方体
    position_marker_.type = visualization_msgs::Marker::CUBE;
    
    // 设置 marker 动作，ADD 为添加，也可用 MODIFY 或 DELETE
    position_marker_.action = visualization_msgs::Marker::ADD;
    
    // 设置位置，从 position_odometry_ 中获取
    position_marker_.pose.position.x = position_odometry_.pose.pose.position.x;
    position_marker_.pose.position.y = position_odometry_.pose.pose.position.y;
    position_marker_.pose.position.z = position_odometry_.pose.pose.position.z;
    
    // 设置姿态
    position_marker_.pose.orientation.x = position_odometry_.pose.pose.orientation.x;
    position_marker_.pose.orientation.y = position_odometry_.pose.pose.orientation.y;
    position_marker_.pose.orientation.z = position_odometry_.pose.pose.orientation.z;
    position_marker_.pose.orientation.w = position_odometry_.pose.pose.orientation.w;
    
    // 设置 marker 的尺寸（假设车的长宽高为2m x 1m x 0.5m）
    position_marker_.scale.x = 4.90;
    position_marker_.scale.y = 1.93;
    position_marker_.scale.z = 1.62;
    
    // 设置颜色和透明度（绿色）
    position_marker_.color.r = 0.0;
    position_marker_.color.g = 1.0;
    position_marker_.color.b = 0.0;
    position_marker_.color.a = 1.0; // 不透明
}

void Localization::Publish(){
  AdapterManager::PublishLocalization(position_odometry_);
  AdapterManager::PublishVehicleLocation(position_marker_);
}

void Localization::Stop() {
  
}

void Localization::loadAndPublishLanelet2Map(const std::string& map_file){

  lanelet::ErrorMessages errors;
  auto projector = lanelet::projection::UtmProjector(lanelet::Origin({49.0, 8.0}));

  // Load the map
  lanelet::LaneletMapPtr map = lanelet::load(map_file, projector, &errors);

  if (!errors.empty()) {
      for (const auto& error : errors) {
          ROS_ERROR("%s", error.c_str());
      }
      return;
  }

  int id = 0;

  // Clear existing markers
  lanelet2_map_.markers.clear();

  // Convert lanelets to visualization markers
  for (const auto& lanelet : map->laneletLayer) {
      visualization_msgs::Marker road_strip;
      road_strip.header.frame_id = "map";
      road_strip.header.stamp = ros::Time::now();
      road_strip.ns = "lanelets";
      road_strip.id = id++;
      road_strip.type = visualization_msgs::Marker::TRIANGLE_LIST;
      road_strip.action = visualization_msgs::Marker::ADD;
      road_strip.color.a = 1.0;  // Alpha value to make it visible
      road_strip.color.r = 0.5;  // Color red
      road_strip.color.g = 0.5;  // Color green
      road_strip.color.b = 0.5;  // Color blue
      road_strip.scale.x = 1.0;  // Set scale to avoid scale zero errors
      road_strip.scale.y = 1.0;
      road_strip.scale.z = 1.0;

      // Set orientation to avoid uninitialized quaternion warning
      road_strip.pose.orientation.w = 1.0;
      road_strip.pose.orientation.x = 0.0;
      road_strip.pose.orientation.y = 0.0;
      road_strip.pose.orientation.z = 0.0;

      auto left_points = lanelet.leftBound2d();
      auto right_points = lanelet.rightBound2d();

      for (size_t i = 0; i < left_points.size() - 1; ++i) {
          addTriangleToList(road_strip.points, left_points[i], right_points[i], left_points[i + 1]);
          addTriangleToList(road_strip.points, right_points[i], right_points[i + 1], left_points[i + 1]);
      }

      lanelet2_map_.markers.push_back(road_strip);
  }

  // Debug: Add a fixed marker for communication verification
  visualization_msgs::Marker debug_marker;
  debug_marker.header.frame_id = "map";
  debug_marker.header.stamp = ros::Time::now();
  debug_marker.ns = "debug";
  debug_marker.id = id++;
  debug_marker.type = visualization_msgs::Marker::SPHERE;
  debug_marker.action = visualization_msgs::Marker::ADD;
  debug_marker.pose.position.x = 0.0;
  debug_marker.pose.position.y = 0.0;
  debug_marker.pose.position.z = 0.0;
  debug_marker.pose.orientation.w = 1.0;  // Set orientation to avoid uninitialized quaternion warning
  debug_marker.pose.orientation.x = 0.0;
  debug_marker.pose.orientation.y = 0.0;
  debug_marker.pose.orientation.z = 0.0;
  debug_marker.scale.x = 1.0;
  debug_marker.scale.y = 1.0;
  debug_marker.scale.z = 1.0;
  debug_marker.color.a = 1.0; // Alpha
  debug_marker.color.r = 1.0; // Red
  debug_marker.color.g = 0.0; // Green
  debug_marker.color.b = 0.0; // Blue

  lanelet2_map_.markers.push_back(debug_marker);
}

void Localization::addTriangleToList(std::vector<geometry_msgs::Point>& points,
                       const lanelet::BasicPoint2d& p1, const lanelet::BasicPoint2d& p2, const lanelet::BasicPoint2d& p3) {
    geometry_msgs::Point gp1, gp2, gp3;
    gp1.x = p1.x(); gp1.y = p1.y(); gp1.z = 0;
    gp2.x = p2.x(); gp2.y = p2.y(); gp2.z = 0;
    gp3.x = p3.x(); gp3.y = p3.y(); gp3.z = 0;
    points.push_back(gp1);
    points.push_back(gp2);
    points.push_back(gp3);
}

void Localization::PCDConvert(const std::string& file_path, bool apply_filter, bool save_filtered, const std::string& save_path) {
    // Load the PCD file
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_path, *cloud) == -1) {
        PCL_ERROR("Couldn't read file %s \n", file_path.c_str());
        return;
    }

    // std::cout << "Original point cloud size: " << cloud->points.size() << std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    if (apply_filter) {
        // Create the filtering objects: downsample the dataset using a smaller leaf size
        pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
        voxel_filter.setInputCloud(cloud);
        voxel_filter.setLeafSize(0.2f, 0.2f, 0.2f); // Adjust the leaf size
        voxel_filter.filter(*cloud_filtered);

        // std::cout << "Downsampled point cloud size: " << cloud_filtered->points.size() << std::endl;

        // Remove statistical outliers
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(cloud_filtered);
        sor.setMeanK(50);  // Number of nearest neighbors to use for mean distance estimation
        sor.setStddevMulThresh(1.0);  // Standard deviation multiplier for distance threshold
        sor.filter(*cloud_filtered);

        // std::cout << "Denoised point cloud size: " << cloud_filtered->points.size() << std::endl;

        if (save_filtered && !save_path.empty()) {
            // Save the filtered point cloud to a file
            pcl::io::savePCDFileASCII(save_path, *cloud_filtered);
            std::cout << "Saved filtered point cloud to " << save_path << std::endl;
        }
    } else {
        cloud_filtered = cloud; // If not applying filter, use the original cloud
    }

    // Convert to ROS data type
    pcl::toROSMsg(*cloud_filtered, cloud_point_map_);
    cloud_point_map_.header.frame_id = "map";
}

} // namespace localization
} // namespace EDrive