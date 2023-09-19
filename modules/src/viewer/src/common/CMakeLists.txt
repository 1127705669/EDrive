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

set(VIEWER_AGENT_SRC
  viewer_agent.cc
)

add_library(viewer_agent SHARED ${VIEWER_AGENT_SRC})

target_link_libraries(viewer_agent 
  ${PROTOBUF_LIBRARIES}
  ${catkin_LIBRARIES}
  viewer_proto
)

add_library(visual_common INTERFACE)

target_link_libraries(visual_common INTERFACE
  viewer_agent
)