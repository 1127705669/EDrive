cmake_minimum_required(VERSION 3.0.2)
project(viewer_agent)

find_package(glog REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

###################################
#         viewer_agent            #
###################################
add_library(viewer_agent INTERFACE)

target_link_libraries(viewer_agent INTERFACE
  perception_agent
)

###################################
#        perception_agent         #
###################################
add_library(perception_agent SHARED 
  perception_agent.cc
)

target_include_directories(perception_agent PUBLIC
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${THIRD_PARTY_DIR}/glog/include
  ${THIRD_PARTY_DIR}/gflags/include
)

target_link_libraries(perception_agent 
  ${PROTOBUF_LIBRARIES}
  ${catkin_LIBRARIES}
  glog::glog
  viewer_proto
)