cmake_minimum_required(VERSION 3.0.2)
project(common)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

include_directories(
include
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}
)

set(VIEWER_MANAGER_SRC 
  viewer_manager.cc
)

add_library(viewer_common SHARED ${VIEWER_MANAGER_SRC})