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
)

target_link_libraries(routing_result_generator 
  common_adapters
  graph
  routing_proto
)
