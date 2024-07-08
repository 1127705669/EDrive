cmake_minimum_required(VERSION 3.0.2)
project(routing_core)

find_package(Protobuf REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

###################################
#    routing_result_generator     #
###################################
add_library(routing_result_generator SHARED 
  result_generator.cc
)

target_include_directories(routing_result_generator PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)

target_link_libraries(routing_result_generator 
  common_adapters
  graph
  routing_proto
)

###################################
#routing_black_list_range_generator#
###################################
add_library(routing_black_list_range_generator SHARED 
  black_list_range_generator.cc
)

target_include_directories(routing_black_list_range_generator PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)

target_link_libraries(routing_result_generator 
  graph
)

###################################
#        routing_navigator        #
###################################
add_library(routing_navigator SHARED 
  navigator.cc
)

target_include_directories(routing_navigator PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)

target_link_libraries(routing_navigator 
  routing_black_list_range_generator
  routing_result_generator
  common_adapters
  common_proto
  util
  routing_gflags
  graph
  routing_proto
  strategy
)