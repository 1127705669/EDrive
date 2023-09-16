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

set(VIEWER_AGENT_SRC
  viewer_agent.cc
)

add_library(viewer_agent SHARED ${VIEWER_AGENT_SRC})

target_link_libraries(viewer_agent 
  ${PROTOBUF_LIBRARIES}
  ${catkin_LIBRARIES}
  viewer_proto
)

add_dependencies(viewer_agent ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})