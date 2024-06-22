/******************************************************************************
 * Copyright 2023 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

# pragma once

#include <ros/ros.h>

#include <fstream>
#include <iostream>
#include <fcntl.h>
#include <unistd.h>

#include "google/protobuf/io/zero_copy_stream_impl.h"
#include "google/protobuf/text_format.h"

namespace EDrive {
namespace common {
namespace util {

std::string GetRootPath() {
  std::string ret_value;
  char buffer[FILENAME_MAX];
    
  if (getcwd(buffer, sizeof(buffer)) == nullptr) {
    ROS_ERROR("Cannot get root path!");
  }
  ret_value = buffer;

  return ret_value;
}

template  <typename MessageType>
void GetProtoFromASIIFile(const std::string &adapter_config_filename, MessageType *message ) {
  using google::protobuf::TextFormat;
  using google::protobuf::io::FileInputStream;
  using google::protobuf::io::ZeroCopyInputStream;

  int file_descriptor = open(adapter_config_filename.c_str(), O_RDONLY);
  if (file_descriptor < 0) {
    ROS_ERROR("Failed to open file in text mode.");
  }

  ZeroCopyInputStream *input = new FileInputStream(file_descriptor);
  bool success = TextFormat::Parse(input, message);
  if (!success) {
    ROS_ERROR("Failed to parse file as text proto.");
  }
  delete input;
  close(file_descriptor);
}

}  //  namespace util
}  //  namespace common
}  //  namespace EDrive