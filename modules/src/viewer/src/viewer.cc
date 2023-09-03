/******************************************************************************
 * Copyright 2022 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#include <ros/ros.h>
#include "viewer/src/viewer.h"
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

namespace EDrive {
namespace viewer {

void imageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    // 在这里处理深度图像消息
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // 显示深度图像
    cv::imshow("Depth Image", cv_ptr->image);
    cv::waitKey(1);
}

std::string Viewer::Name() const { return "viewer"; }

EDrive::Result_state Viewer::Init(){
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe<sensor_msgs::Image>("/carla/ego_vehicle/rgb_front/image", 1, imageCallback);

  ros::spin();
  return State_Ok;
}

EDrive::Result_state Viewer::Start(){
  return State_Ok;
}

void Viewer::Stop() {
  
}

} // namespace viewer
} // namespace EDrive