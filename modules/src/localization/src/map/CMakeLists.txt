cmake_minimum_required(VERSION 3.0.2)
project(localization)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  visualization_msgs
  geometry_msgs
)

include_directories(
include
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${tinyxml2_INCLUDE_DIRS}
)

###################################
#           vector_map            #
###################################

set(VECTOR_MAP_SRC
  vector_map.cc
)

add_library(vector_map SHARED
  ${VECTOR_MAP_SRC}
)

add_dependencies(vector_map ${PROJECT_NAME}_generate_messages_cpp)

target_link_libraries(vector_map
  ${PROTOBUF_LIBRARIES}
  ${catkin_LIBRARIES}
  tinyxml2
)