/******************************************************************************
 * Copyright 2022 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#include <ros/ros.h>
#include "localization/src/localization.h"

#include "localization/src/common/localization_gflags.h"
#include "common/util/file.h"
#include "common/adapters/adapter_manager.h"

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

  EDrive::common::util::GetProtoFromASCIIFile(localization_conf_file, &localization_conf_);
  
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

  vector_mapper_.publishMiddlePath(
    {21960, 24426, 21623, 34728, 21311, 27800, 20942, 29426, 20507, 32319, 20200, 3715, 23683, 3624, 22666, 30336, 22329,32908}, 
    path_, trajectory_pb_);
    
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

  Publish();
}

void Localization::Publish(){
  AdapterManager::PublishLocalization(position_odometry_);
  AdapterManager::PublishFixedPath(path_);
  AdapterManager::PublishPlanning(trajectory_pb_);
}

void Localization::Stop() {
  
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